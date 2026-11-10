"""Start one headless PX4 SIH quadx instance per scenario vehicle.

Each instance i spawns at its vehicle's start point (PX4_HOME_LAT/LON) and
streams offboard MAVLink to UDP 14540+i. Runs until Ctrl-C / SIGTERM, then
stops every instance.

    python launch_sih.py --scenario three_way_swap --out runs/<dir>
"""
import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import geo
import scenarios

PX4_DIR = Path(os.environ.get("PX4_DIR", "~/PX4-Autopilot")).expanduser()
BUILD = PX4_DIR / "build/px4_sitl_default"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True, choices=list(scenarios.SCENARIOS))
    ap.add_argument("--out", required=True, help="run directory for PX4 logs")
    args = ap.parse_args()

    if not (BUILD / "bin/px4").exists():
        sys.exit(f"PX4 SITL not built: {BUILD}/bin/px4 (run `make px4_sitl_default` in {PX4_DIR})")
    subprocess.run(["pkill", "-x", "px4"], check=False)
    time.sleep(1)

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    procs = []
    for i, v in enumerate(scenarios.SCENARIOS[args.scenario]["vehicles"]):
        lat, lon = geo.enu_to_latlon(v["start"][0], v["start"][1])
        wd = BUILD / f"instance_{i}"
        # Fresh params each run so nothing leaks between scenarios.
        subprocess.run(["rm", "-rf", str(wd)], check=False)
        wd.mkdir(parents=True)
        env = dict(os.environ, PX4_SYS_AUTOSTART="10040", PX4_SIM_MODEL="sihsim_quadx",
                   PX4_HOME_LAT=f"{lat:.8f}", PX4_HOME_LON=f"{lon:.8f}", PX4_HOME_ALT="0")
        log = open(out / f"px4_{i}.log", "w")
        p = subprocess.Popen([str(BUILD / "bin/px4"), "-i", str(i), "-d", str(BUILD / "etc")],
                             cwd=wd, env=env, stdout=log, stderr=subprocess.STDOUT)
        procs.append(p)
        print(f"instance {i} ({v['v_id']}) pid {p.pid} home {lat:.7f},{lon:.7f} -> udp 1454{i}", flush=True)

    def stop(*_):
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        print("PX4 instances stopped", flush=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    while True:
        dead = [i for i, p in enumerate(procs) if p.poll() is not None]
        if dead:
            print(f"instance(s) {dead} exited; see {out}/px4_<i>.log", flush=True)
            stop()
        time.sleep(1)


if __name__ == "__main__":
    main()
