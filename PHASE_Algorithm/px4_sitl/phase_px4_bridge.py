"""Fly a PHASE scenario on PX4 SIH quadcopters, closed loop.

Each 0.2 s tick (PHASE's dt): read every drone's estimated position, hand it
to PHASE as ``current_pos``, run one PHASE tick, and send the resulting target
to PX4 as an offboard position + velocity-feedforward setpoint.

Start the drones first with launch_sih.py (same --scenario), then:

    python phase_px4_bridge.py --scenario three_way_swap --out runs/<dir>
"""
import argparse
import asyncio
import csv
import json
import os
import subprocess
import sys
import time
import traceback
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore", category=FutureWarning)  # mavsdk -> mavsdk-grpc rename notice
from mavsdk import System  # noqa: E402
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw  # noqa: E402

import geo  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # PHASE_Algorithm/, home of phase_d
from phase_d import scenarios  # noqa: E402
from phase_d.phase_dynamic import T_LATENCY, TAU_RESPONSE, make_dynamic_stepper_class  # noqa: E402
from phase_d.phase_stepper import load_phase, make_stepper_class  # noqa: E402

VARIANTS = {"v12": make_stepper_class, "phase_d": make_dynamic_stepper_class}

VEL_CLAMP_FACTOR = 1.5  # max commanded speed = factor * vehicle speed
SERVERS = []


def start_mavsdk_servers(n, log_dir):
    """Start one mavsdk_server per drone BEFORE any gRPC channel exists.

    MAVSDK's own connect() forks mavsdk_server from inside a process where gRPC
    is already running for the previous drone. gRPC then warns "Other threads
    are currently calling into gRPC, skipping fork() handlers", and connect()
    took ~60 s (H-11). Spawning the servers up front makes setup ~1 s."""
    import importlib.resources as ir
    import mavsdk.bin
    exe = os.fspath(ir.files(mavsdk.bin) / "mavsdk_server")
    for i in range(n):
        log = open(Path(log_dir) / f"mavsdk_server_{i}.log", "w")
        SERVERS.append(subprocess.Popen([exe, "-p", str(50051 + i), f"udpin://0.0.0.0:{14540 + i}"],
                                        stdout=log, stderr=subprocess.STDOUT))


def stop_mavsdk_servers():
    for p in SERVERS:
        p.kill()
STATE = {"phase_started": False}


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]])


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]])


class Drone:
    def __init__(self, idx, cfg):
        self.idx = idx
        self.v_id = cfg["v_id"]
        self.cfg = cfg
        # Attach to a mavsdk_server we started ourselves (start_mavsdk_servers).
        self.sys = System(mavsdk_server_address="127.0.0.1", port=50051 + idx)
        self.local_ned = None      # latest local NED position
        self.local_vel = None
        self.global_enu = None     # latest lat/lon/rel-alt converted to shared ENU
        self.origin_enu = None     # shared-ENU position of this drone's local NED origin
        self.flight_mode = None
        self.in_air = False
        self.sim_time = None       # PX4 time (s) from odometry; SIH's clock

    def enu(self):
        """Measured position in PHASE's shared frame."""
        return self.origin_enu + ned_to_enu(self.local_ned)

    def to_local_ned(self, enu):
        return enu_to_ned(np.asarray(enu) - self.origin_enu)

    async def connect(self):
        await self.sys.connect()
        async for s in self.sys.core.connection_state():
            if s.is_connected:
                break
        await self.sys.telemetry.set_rate_position_velocity_ned(20.0)
        await self.sys.telemetry.set_rate_position(10.0)
        await self.sys.telemetry.set_rate_odometry(50.0)
        asyncio.ensure_future(self._odom())
        asyncio.ensure_future(self._pv())
        asyncio.ensure_future(self._pos())
        asyncio.ensure_future(self._mode())
        asyncio.ensure_future(self._in_air())

    async def _odom(self):
        async for o in self.sys.telemetry.odometry():
            self.sim_time = o.time_usec / 1e6

    async def _pv(self):
        async for pv in self.sys.telemetry.position_velocity_ned():
            self.local_ned = np.array([pv.position.north_m, pv.position.east_m, pv.position.down_m])
            self.local_vel = np.array([pv.velocity.north_m_s, pv.velocity.east_m_s, pv.velocity.down_m_s])

    async def _pos(self):
        async for p in self.sys.telemetry.position():
            e, n = geo.latlon_to_enu(p.latitude_deg, p.longitude_deg)
            self.global_enu = np.array([e, n, p.relative_altitude_m])

    async def _mode(self):
        async for m in self.sys.telemetry.flight_mode():
            self.flight_mode = str(m)

    async def _in_air(self):
        async for a in self.sys.telemetry.in_air():
            self.in_air = a

    async def wait_ready(self, timeout=90):
        t0 = time.time()
        async for h in self.sys.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok and h.is_local_position_ok and h.is_armable:
                return
            if time.time() - t0 > timeout:
                raise RuntimeError(f"{self.v_id}: not ready after {timeout}s ({h})")

    async def calibrate_origin(self, seconds=2.0):
        """origin = global ENU - local ENU, averaged while hovering."""
        samples = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            if self.local_ned is not None and self.global_enu is not None:
                samples.append(self.global_enu - ned_to_enu(self.local_ned))
            await asyncio.sleep(0.1)
        self.origin_enu = np.mean(samples, axis=0)

    async def send(self, enu_pos, enu_vel=(0.0, 0.0, 0.0)):
        n, e, d = self.to_local_ned(enu_pos)
        vn, ve, vd = enu_to_ned(np.asarray(enu_vel, dtype=float))
        await self.sys.offboard.set_position_velocity_ned(
            PositionNedYaw(float(n), float(e), float(d), 0.0),
            VelocityNedYaw(float(vn), float(ve), float(vd), 0.0))


async def wait_until(pred, timeout, what):
    t0 = time.time()
    while not pred():
        if time.time() - t0 > timeout:
            raise RuntimeError(f"timeout waiting for {what}")
        await asyncio.sleep(0.2)


async def fly(args):
    cfg = scenarios.SCENARIOS[args.scenario]
    alt = cfg["vehicles"][0]["start"][2]
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    mod = load_phase()
    if args.variant == "phase_d":
        Stepper = make_dynamic_stepper_class(mod, t_latency=args.t_latency, tau_response=args.tau_response,
                                             use_grid=not args.no_grid)
    else:
        Stepper = make_stepper_class(mod)
    sim = scenarios.build(mod, Stepper, args.scenario)
    print(f"[bridge] variant {args.variant}", flush=True)
    sim.build_original_paths()
    vehicles = {v.v_id: v for v in sim.vehicles}

    start_mavsdk_servers(len(cfg["vehicles"]), out)
    drones = [Drone(i, v) for i, v in enumerate(cfg["vehicles"])]
    print(f"[bridge] PHASE {mod.__phase_path__} (sha256 {mod.__phase_sha256__[:12]})", flush=True)
    # A MAVSDK setup call can occasionally never return even though PX4 is ready
    # (seen in run 20261110_160830_phase_d). Bound the whole setup phase so the
    # runner's retry kicks in after 2 minutes instead of the 10-minute watchdog.
    try:
        await asyncio.wait_for(asyncio.gather(*(d.connect() for d in drones)), 60)
        await asyncio.wait_for(asyncio.gather(*(d.wait_ready() for d in drones)), 60)
    except asyncio.TimeoutError:
        raise RuntimeError("MAVSDK connect/health setup did not complete within 2 minutes")
    print("[bridge] all drones connected and armable", flush=True)

    # Take off to the test altitude, each from its own spawn (= scenario start).
    for d in drones:
        await d.sys.action.set_takeoff_altitude(alt)
        await d.sys.action.arm()
        await d.sys.action.takeoff()
    try:
        await wait_until(lambda: all(d.global_enu is not None and d.global_enu[2] > alt - 0.6 for d in drones),
                         60, "takeoff")
    except RuntimeError:
        raise RuntimeError("takeoff did not complete: " + ", ".join(
            f"{d.v_id} mode={d.flight_mode} alt={None if d.global_enu is None else round(float(d.global_enu[2]), 1)}"
            for d in drones))
    await asyncio.sleep(2.0)
    await asyncio.gather(*(d.calibrate_origin() for d in drones))
    for d in drones:
        print(f"[bridge] {d.v_id} local origin in shared ENU: {np.round(d.origin_enu, 2)}", flush=True)

    # Enter offboard holding exactly the scenario start point, then settle.
    for d in drones:
        start = np.array(d.cfg["start"], dtype=float)
        for _ in range(5):
            await d.send(start)
        try:
            await d.sys.offboard.start()
        except OffboardError as e:
            raise RuntimeError(f"{d.v_id}: offboard start failed: {e._result.result}")
    t_settle = time.time() + 30
    while time.time() < t_settle:
        for d in drones:
            await d.send(np.array(d.cfg["start"], dtype=float))
        if all(np.linalg.norm(d.enu() - np.array(d.cfg["start"])) < 0.3 for d in drones):
            break
        await asyncio.sleep(0.1)
    for d in drones:
        print(f"[bridge] {d.v_id} settled at {np.round(d.enu(), 2)} (start {d.cfg['start']})", flush=True)

    # ---------------- closed loop ----------------
    dt = sim.dt
    ideal = max(v.orig_time for v in sim.vehicles)
    timeout = args.timeout or (6 * ideal + 30)
    rows, overruns, clamps = [], 0, 0
    print(f"[bridge] PHASE loop at {1/dt:.0f} Hz, ideal {ideal:.1f}s, timeout {timeout:.0f}s", flush=True)

    if any(d.flight_mode != "OFFBOARD" for d in drones):
        raise RuntimeError(f"not all drones in offboard before the PHASE run: {[(d.v_id, d.flight_mode) for d in drones]}")
    STATE["phase_started"] = True
    clock = lambda: drones[0].sim_time  # noqa: E731  (all SIH instances run at the same rate)
    s0 = clock()
    t0 = time.monotonic()
    tick = 0
    all_done_since = None
    while True:
        tick += 1
        measured = {d.v_id: d.enu() for d in drones}
        measured_vel = {d.v_id: ned_to_enu(d.local_vel) for d in drones}
        targets = sim.step(measured, measured_vel=measured_vel)
        info = sim.tick_log[-1]
        for m in info["messages"]:
            print(f"[phase t={sim.t:.1f}] {m}", flush=True)

        for d in drones:
            v = vehicles[d.v_id]
            meas = measured[d.v_id]
            if v.reached_goal:
                goal = np.array(v.end, dtype=float)
                goal[2] = v.locked_altitude
                cmd, vel, clamped = goal, np.zeros(3), False
            else:
                cmd = targets[d.v_id]
                # PHASE-D commands an acceleration-bounded velocity itself;
                # V1.2 only gives a position, so derive the feedforward from it.
                vel = np.copy(v.cmd_vel) if args.variant == "phase_d" else (cmd - meas) / dt
                vmax = VEL_CLAMP_FACTOR * v.speed
                spd = np.linalg.norm(vel)
                clamped = spd > vmax
                if clamped:
                    vel *= vmax / spd
                    clamps += 1
            await d.send(cmd, vel)
            gt = d.global_enu
            rows.append([round(sim.t, 2), round(clock() - s0, 3), round(time.monotonic() - t0, 3), d.v_id, v.state, int(v.reached_goal),
                         *np.round(cmd, 3), *np.round(meas, 3), *np.round(vel, 3), int(clamped),
                         *np.round(gt, 3), d.flight_mode])

        if any(d.flight_mode != "OFFBOARD" for d in drones):
            print(f"[bridge] ABORT: left offboard ({[(d.v_id, d.flight_mode) for d in drones]})", flush=True)
            break
        if sim.all_reached():
            all_done_since = all_done_since or time.monotonic()
            if time.monotonic() - all_done_since > 2.0:
                print(f"[bridge] all vehicles reached goals at PHASE t={sim.t:.1f}s", flush=True)
                break
        else:
            all_done_since = None
        if sim.t > timeout:
            print(f"[bridge] TIMEOUT at t={sim.t:.1f}s", flush=True)
            break

        # Pace on PX4's simulation clock, not the wall clock: SIH on macOS runs
        # slower than real time (~0.83x), and PHASE's dt must match the time
        # the vehicles actually flew.
        next_s = s0 + tick * dt
        if clock() - next_s > dt / 2:
            overruns += 1
        while clock() < next_s:
            await asyncio.sleep(0.004)

    # Hold, then land everyone.
    for _ in range(10):
        for d in drones:
            goal = np.array(d.cfg["end"], dtype=float)
            await d.send(goal if vehicles[d.v_id].reached_goal else d.enu())
        await asyncio.sleep(0.1)
    for d in drones:
        await d.sys.action.land()
    try:
        await wait_until(lambda: all(not d.in_air for d in drones), 60, "landing")
        print("[bridge] all landed", flush=True)
    except RuntimeError as e:
        print(f"[bridge] {e}", flush=True)

    # ---------------- outputs ----------------
    header = ["t", "sim_s", "wall_s", "v_id", "state", "reached", "cmd_x", "cmd_y", "cmd_z", "meas_x", "meas_y", "meas_z",
              "vel_x", "vel_y", "vel_z", "vel_clamped", "gps_x", "gps_y", "gps_z", "flight_mode"]
    with open(out / "flight.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    with open(out / "phase_ticks.json", "w") as f:
        json.dump(sim.tick_log, f, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    px4_ver = subprocess.run(["git", "-C", str(Path("~/PX4-Autopilot").expanduser()), "describe", "--tags"],
                             capture_output=True, text=True).stdout.strip()
    meta = {
        "scenario": args.scenario, "config": cfg, "dt": dt,
        "phase_source": mod.__phase_path__, "phase_sha256": mod.__phase_sha256__,
        "px4_version": px4_ver, "sim": "PX4 SIH sihsim_quadx (10040)",
        "ticks": tick, "overruns": overruns, "vel_clamps": clamps,
        "sim_over_wall": round((clock() - s0) / max(time.monotonic() - t0, 1e-6), 3),
        "reached": {v.v_id: bool(v.reached_goal) for v in sim.vehicles},
        "phase_new_time": {v.v_id: v.new_time for v in sim.vehicles},
        "orig_time": {v.v_id: v.orig_time for v in sim.vehicles},
        "variant": args.variant,
        "phase_d_params": ({"t_latency": args.t_latency, "tau_response": args.tau_response,
                            "a_brake": sim.a_brake, "a_cmd": sim.a_cmd, "use_grid": not args.no_grid}
                           if args.variant == "phase_d" else None),
        "counters": dict(getattr(sim, "counters", {})),
        "deflection_events": [{"v_id": e["v_id"], "pos": e["pos"].tolist(), "kind": e.get("kind"),
                               "partner": e.get("partner"), "t": e.get("t"), "dist": e.get("dist")}
                              for e in sim.deflection_events],
        "identified_deadlocks": sorted(map(list, sim.identified_deadlocks)),
        "origins_enu": {d.v_id: d.origin_enu.tolist() for d in drones},
    }
    with open(out / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[bridge] wrote {out}/flight.csv, meta.json, phase_ticks.json", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True, choices=list(scenarios.SCENARIOS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=None, help="PHASE-time limit in seconds")
    ap.add_argument("--watchdog", type=float, default=600, help="wall-clock limit for the whole flight")
    ap.add_argument("--tau-response", type=float, default=TAU_RESPONSE,
                    help="PHASE-D: vehicle response time constant in s (PX4: 1/MPC_XY_VEL_P_ACC)")
    ap.add_argument("--t-latency", type=float, default=T_LATENCY,
                    help="PHASE-D: tick + telemetry latency in s")
    ap.add_argument("--no-grid", action="store_true", help="PHASE-D: O(N^2) neighbour search instead of the grid")
    ap.add_argument("--variant", choices=list(VARIANTS), default="v12",
                    help="v12 = PHASE as written; phase_d = growing balls + priority braking + full deflections")
    args = ap.parse_args()
    # MAVSDK's telemetry streams and mavsdk_server threads keep the interpreter
    # alive after the event loop ends, so always exit hard.
    # Exit codes: 0 ok, 3 = PX4/setup failure before PHASE took control
    # (safe to retry), 2 = failure during the PHASE run, 130 = interrupted.
    code = 0
    try:
        asyncio.run(asyncio.wait_for(fly(args), timeout=args.watchdog))
    except KeyboardInterrupt:
        code = 130
    except Exception:
        traceback.print_exc()
        code = 2 if STATE["phase_started"] else 3
        print(f"[bridge] FAILED {'during' if code == 2 else 'before'} the PHASE run (exit {code})", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    stop_mavsdk_servers()  # os._exit doesn't take child processes with it
    os._exit(code)


if __name__ == "__main__":
    main()
