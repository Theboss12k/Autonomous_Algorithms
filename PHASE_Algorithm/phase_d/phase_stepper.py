"""Tick-by-tick driver for PHASE so it can run in closed loop with real vehicles.

PHASE's ``Simulation.run()`` is a single monolithic loop that integrates its own
point-mass vehicles. To fly PX4 drones we need one tick at a time, with each
vehicle's ``current_pos`` taken from *measured* telemetry instead of the last
commanded position.

``load_phase()`` imports the PHASE source file unchanged (the filenames contain
dots, so a normal ``import`` can't be used). ``make_stepper_class()`` builds a
subclass of that module's ``Simulation`` whose ``step()`` is the body of the
``run()`` while-loop, copied phase for phase. ``test_stepper_equivalence.py``
checks that with perfect tracking ``step()`` reproduces ``run()`` exactly, so
any drift between this copy and the algorithm is caught.
"""
from __future__ import annotations

import collections
import contextlib
import hashlib
import importlib.util
import io
import os
import time
from pathlib import Path

import numpy as np

VERSIONS_DIR = Path(__file__).resolve().parents[1] / "versions"
DEFAULT_VERSION = "V1.2"

SAFETY_BUFFER = 0.2  # same constant as Simulation.run()


def version_path(version: str) -> Path:
    """Altitude-locked source of a PHASE version in this repo, e.g. ``V1.2``."""
    return VERSIONS_DIR / version / "phase_altitude_locked.py"


def phase_source() -> Path:
    """PHASE source to run: ``$PHASE_SRC`` (a version name such as ``V1.1`` or a
    path to any PHASE file), else the newest version in ``versions/``."""
    choice = os.environ.get("PHASE_SRC", DEFAULT_VERSION)
    if (VERSIONS_DIR / choice).is_dir():
        return version_path(choice)
    return Path(choice).expanduser().resolve()


DEFAULT_PHASE_SRC = version_path(DEFAULT_VERSION)


def load_phase(path: Path | None = None):
    path = Path(path) if path else phase_source()
    if not path.exists():
        raise FileNotFoundError(f"PHASE source not found: {path} (set PHASE_SRC)")
    spec = importlib.util.spec_from_file_location("phase_algorithm", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.__phase_path__ = str(path)
    mod.__phase_sha256__ = hashlib.sha256(path.read_bytes()).hexdigest()
    return mod


def make_stepper_class(mod):
    Simulation = mod.Simulation
    calculate_90_deg_deflection = mod.calculate_90_deg_deflection

    class PhaseStepper(Simulation):
        """``Simulation`` that advances one ``dt`` per ``step()`` call."""

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.t = 0.0
            self.tick_log = []  # per-tick diagnostics for analysis

        def build_original_paths(self):
            # Same as the head of Simulation.run().
            for v in self.vehicles:
                steps = int(np.ceil(v.orig_time / self.dt))
                for i in range(steps + 1):
                    fraction = min((i * self.dt) / v.orig_time, 1.0)
                    original_pos = v.start + fraction * (v.end - v.start)
                    if not self.allow_3d:
                        original_pos[2] = v.locked_altitude
                    v.original_path.append({'t': v.start_time + (i * self.dt), 'pos': original_pos})

        def all_reached(self):
            return all(v.reached_goal for v in self.vehicles)

        def step(self, measured=None, quiet=True, measured_vel=None):
            """Advance one tick.

            ``measured``: optional {v_id: xyz} of where each vehicle really is.
            When given, it replaces ``current_pos`` before PHASE runs, closing
            the loop. ``measured_vel`` ({v_id: xyz velocity}) is stored on the
            vehicles as ``vel`` for variants that model dynamics; V1.2 ignores
            it. Returns {v_id: target xyz} (PHASE's committed position).
            """
            if measured:
                for v in self.vehicles:
                    if v.v_id in measured:
                        v.current_pos = np.array(measured[v.v_id], dtype=float)
            if measured_vel:
                for v in self.vehicles:
                    if v.v_id in measured_vel:
                        v.vel = np.array(measured_vel[v.v_id], dtype=float)

            t0 = time.perf_counter()
            sink = io.StringIO()
            with contextlib.redirect_stdout(sink) if quiet else contextlib.nullcontext():
                info = self._tick()
            info['compute_s'] = time.perf_counter() - t0
            info['messages'] = [m for m in sink.getvalue().splitlines() if m.strip()] if quiet else []
            self.tick_log.append(info)
            return {v.v_id: np.copy(v.current_pos) for v in self.vehicles}

        # ------------------------------------------------------------------
        # Body of Simulation.run()'s while-loop (PHASE V1.2), one iteration.
        # Keep in step with the source; the equivalence test enforces it.
        # ------------------------------------------------------------------
        def _tick(self):
            self.t += self.dt
            t = self.t
            safety_buffer = SAFETY_BUFFER
            astar_calls = 0

            # PHASE 1: NORMAL EXECUTION
            for v in self.vehicles:
                if not v.is_active:
                    if t >= v.start_time:
                        v.is_active = True
                    else:
                        v.prop_pos = np.copy(v.current_pos)
                        continue

                if not self.allow_3d:
                    v.current_pos[2] = v.locked_altitude
                    v.prop_pos[2] = v.locked_altitude
                    v.last_pos[2] = v.locked_altitude
                    effective_goal = np.copy(v.end)
                    effective_goal[2] = v.locked_altitude
                else:
                    effective_goal = v.end

                dist_to_goal = np.linalg.norm(effective_goal - v.current_pos)

                if dist_to_goal < 0.5:
                    v.reached_goal = True
                    v.state = 'NORMAL'
                    v.prop_pos = np.copy(v.current_pos)
                    continue
                else:
                    v.reached_goal = False

                if v.state == 'DEFLECTING':
                    dist_to_target = np.linalg.norm(v.deflection_target - v.current_pos)
                    if dist_to_target > 0.5:
                        direction = (v.deflection_target - v.current_pos) / dist_to_target
                        if not self.allow_3d: direction[2] = 0.0
                        v.last_dir = direction
                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, dist_to_target)
                        if not self.allow_3d: v.prop_pos[2] = v.locked_altitude
                    else:
                        v.prop_pos = np.copy(v.current_pos)
                        if not self.allow_3d: v.prop_pos[2] = v.locked_altitude
                    continue

                if v.state == 'NORMAL':
                    if v.needs_replan:
                        dynamic_walls = []
                        for other in self.vehicles:
                            if other.v_id != v.v_id and not other.reached_goal and other.is_active:
                                if np.linalg.norm(other.current_pos - v.current_pos) <= self.astar_frame_size:
                                    dynamic_walls.append((np.copy(other.current_pos), other.margin))

                        path = self.astar.compute_path(v, v.current_pos, v.end, self.obstacles, dynamic_walls)
                        astar_calls += 1

                        if not self.allow_3d:
                            path = [np.array([p[0], p[1], v.locked_altitude], dtype=float) for p in path]
                        v.waypoints = collections.deque(path)
                        v.needs_replan = False

                    if len(v.waypoints) > 0:
                        target = v.waypoints[0]
                        vec = target - v.current_pos
                        dist = np.linalg.norm(vec)
                        if dist < 0.5:
                            v.waypoints.popleft()
                            if len(v.waypoints) > 0:
                                target = v.waypoints[0]
                                vec = target - v.current_pos
                                dist = np.linalg.norm(vec)

                        if dist > 1e-5:
                            direction = vec / dist
                            if not self.allow_3d: direction[2] = 0.0
                            v.last_dir = direction
                        else:
                            direction = np.zeros(3)

                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, dist)
                        if not self.allow_3d: v.prop_pos[2] = v.locked_altitude
                    else:
                        if not self.allow_3d:
                            fallback_goal = np.copy(v.end)
                            fallback_goal[2] = v.locked_altitude
                        else:
                            fallback_goal = v.end

                        fallback_vec = fallback_goal - v.current_pos
                        fallback_dist = np.linalg.norm(fallback_vec)

                        if fallback_dist > 1e-5:
                            direction = fallback_vec / fallback_dist
                            if not self.allow_3d: direction[2] = 0.0
                            v.last_dir = direction
                        else:
                            direction = np.zeros(3)

                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, fallback_dist)
                        if not self.allow_3d: v.prop_pos[2] = v.locked_altitude

            active_veh = [v for v in self.vehicles if v.is_active and not v.reached_goal]
            n_veh = len(active_veh)

            # PHASE 2: COLLISION / SAFETY-BUBBLE BREACH -> STEERING
            current_conflict_events = set()
            new_conflict_detected = False
            new_pairs = []

            for i in range(n_veh):
                vi = active_veh[i]
                for j in range(i + 1, n_veh):
                    vj = active_veh[j]
                    req_dist = vi.margin + vj.margin + safety_buffer
                    dist = np.linalg.norm(vi.prop_pos - vj.prop_pos)
                    pair_id = tuple(sorted((vi.v_id, vj.v_id)))

                    if dist < req_dist:
                        current_conflict_events.add(pair_id)

                        if pair_id not in self.active_conflict_events:
                            new_conflict_detected = True
                            new_pairs.append(pair_id)

                            mom_i = vi.momentum
                            mom_j = vj.momentum

                            if mom_i == mom_j:
                                if pair_id not in self.identified_deadlocks:
                                    print(f"⚠️ SYMMETRIC DEADLOCK IDENTIFIED: Veh {vi.v_id} & Veh {vj.v_id} have equal momentum. Artificially boosting {vi.v_id} by 50% to prevent lock.")
                                    self.identified_deadlocks.add(pair_id)
                                mom_i *= 1.5

                            sum_mom = mom_i + mom_j

                            if vi.state == 'NORMAL':
                                dir_i = calculate_90_deg_deflection(vi, vj, self.obstacles)
                                dist_i = max((mom_j / sum_mom) * (vi.speed * 4.0), vi.margin * 1.5)
                                vi.deflection_target = vi.current_pos + dir_i * dist_i
                                if not self.allow_3d: vi.deflection_target[2] = vi.locked_altitude
                                vi.state = 'DEFLECTING'
                                vi.waypoints.clear()
                                vi.prop_pos = vi.current_pos + dir_i * min(vi.speed * self.dt, dist_i)
                                if not self.allow_3d: vi.prop_pos[2] = vi.locked_altitude
                                self.deflection_events.append({'v_id': vi.v_id, 'pos': np.copy(vi.current_pos)})

                            if vj.state == 'NORMAL':
                                dir_j = calculate_90_deg_deflection(vj, vi, self.obstacles)
                                dist_j = max((mom_i / sum_mom) * (vj.speed * 4.0), vj.margin * 1.5)
                                vj.deflection_target = vj.current_pos + dir_j * dist_j
                                if not self.allow_3d: vj.deflection_target[2] = vj.locked_altitude
                                vj.state = 'DEFLECTING'
                                vj.waypoints.clear()
                                vj.prop_pos = vj.current_pos + dir_j * min(vj.speed * self.dt, dist_j)
                                if not self.allow_3d: vj.prop_pos[2] = vj.locked_altitude
                                self.deflection_events.append({'v_id': vj.v_id, 'pos': np.copy(vj.current_pos)})

            self.active_conflict_events = current_conflict_events

            # PHASE 3: GLOBAL MICRO-PBD
            pre_pbd = {v.v_id: np.copy(v.prop_pos) for v in active_veh}
            max_pbd_iters = 1000
            pbd_iters = 0

            for _ in range(max_pbd_iters):
                pbd_iters += 1
                for v in active_veh:
                    for obs in self.obstacles:
                        if obs.z_min <= v.prop_pos[2] <= obs.z_max:
                            vec2d = v.prop_pos[:2] - np.array([obs.x, obs.y])
                            dist2d = np.linalg.norm(vec2d)
                            req_dist = obs.radius + v.margin + safety_buffer
                            if dist2d < req_dist:
                                overlap = req_dist - dist2d
                                dir2d = vec2d / dist2d if dist2d > 1e-5 else np.array([1.0, 0.0])
                                v.prop_pos[:2] += dir2d * overlap

                for i in range(n_veh):
                    vi = active_veh[i]
                    for j in range(i + 1, n_veh):
                        vj = active_veh[j]
                        req_dist = vi.margin + vj.margin + safety_buffer
                        delta = vi.prop_pos - vj.prop_pos
                        dist = np.linalg.norm(delta)
                        if dist < req_dist:
                            overlap = req_dist - dist
                            direction = delta / dist if dist > 1e-5 else np.array([1.0, 0.0, 0.0])
                            if not self.allow_3d: direction[2] = 0.0

                            mom_i = vi.momentum
                            mom_j = vj.momentum

                            if mom_i == mom_j:
                                pair_id = tuple(sorted((vi.v_id, vj.v_id)))
                                if pair_id not in self.identified_deadlocks:
                                    print(f"⚠️ SYMMETRIC DEADLOCK IDENTIFIED (PBD): Veh {vi.v_id} & Veh {vj.v_id} have equal momentum. Artificially boosting {vi.v_id} by 50% to prevent lock.")
                                    self.identified_deadlocks.add(pair_id)
                                mom_i *= 1.5

                            sum_mom = mom_i + mom_j

                            vi.prop_pos += direction * overlap * (mom_j / sum_mom) * 0.8
                            vj.prop_pos -= direction * overlap * (mom_i / sum_mom) * 0.8

                if not self.allow_3d:
                    for v in active_veh:
                        v.prop_pos[2] = v.locked_altitude

                all_clear = True
                for v in active_veh:
                    for obs in self.obstacles:
                        if obs.z_min <= v.prop_pos[2] <= obs.z_max:
                            dist2d = np.linalg.norm(v.prop_pos[:2] - np.array([obs.x, obs.y]))
                            req_dist = obs.radius + v.margin + safety_buffer
                            if dist2d < req_dist:
                                all_clear = False
                                break
                    if not all_clear: break

                if all_clear:
                    for i in range(n_veh):
                        vi = active_veh[i]
                        for j in range(i + 1, n_veh):
                            vj = active_veh[j]
                            req_dist = vi.margin + vj.margin + safety_buffer
                            if np.linalg.norm(vi.prop_pos - vj.prop_pos) < req_dist:
                                all_clear = False
                                break
                        if not all_clear: break

                if all_clear: break

            pbd_shift = {vid: float(np.linalg.norm(next(v.prop_pos for v in active_veh if v.v_id == vid) - p))
                         for vid, p in pre_pbd.items()}

            # PHASE 4: GLOBAL A* REPLANNING (only on a new conflict)
            if new_conflict_detected:
                for v in active_veh:
                    pbd_start = np.copy(v.prop_pos)
                    dynamic_walls = []

                    for other in active_veh:
                        if other.v_id != v.v_id and not other.reached_goal:
                            dynamic_walls.append((np.copy(other.prop_pos), other.margin))

                    path = self.astar.compute_path(v, pbd_start, v.end, self.obstacles, dynamic_walls)
                    astar_calls += 1

                    if not self.allow_3d:
                        path = [np.array([p[0], p[1], v.locked_altitude], dtype=float) for p in path]

                    v.waypoints = collections.deque(path)
                    v.needs_replan = False
                    v.state = 'NORMAL'
                    v.deflection_target = None

            # PHASE 5: COMMIT STATE & IDENTIFY POSITIONAL DEADLOCKS
            for v in self.vehicles:
                if not v.is_active: continue

                v.last_pos = np.copy(v.current_pos)
                v.current_pos = np.copy(v.prop_pos)

                if not self.allow_3d:
                    v.current_pos[2] = v.locked_altitude
                    v.prop_pos[2] = v.locked_altitude

                v.resolved_path.append({'t': t, 'pos': np.copy(v.current_pos)})
                v.state_history.append(v.state)

                if not self.allow_3d:
                    effective_goal = np.copy(v.end)
                    effective_goal[2] = v.locked_altitude
                else:
                    effective_goal = v.end

                if np.linalg.norm(v.current_pos - effective_goal) < 0.5:
                    if not v.reached_goal: v.reached_goal = True
                    if v.new_time == 0.0: v.new_time = t
                else:
                    if np.linalg.norm(v.current_pos - v.last_progress_pos) < 0.1:
                        v.stuck_time += self.dt
                        if v.stuck_time > 3.0 and not v.deadlock_reported:
                            print(f"🚨 POSITIONAL DEADLOCK IDENTIFIED: Veh {v.v_id} is stuck at {np.round(v.current_pos, 2)}.")
                            v.deadlock_reported = True
                    else:
                        if v.deadlock_reported:
                            print(f"✅ Veh {v.v_id} has recovered from its positional deadlock.")
                            v.deadlock_reported = False
                        v.last_progress_pos = np.copy(v.current_pos)
                        v.stuck_time = 0.0

            return {
                't': t,
                'new_conflicts': new_pairs,
                'active_conflicts': sorted(current_conflict_events),
                'pbd_iters': pbd_iters,
                'pbd_shift': pbd_shift,
                'astar_calls': astar_calls,
            }

    return PhaseStepper
