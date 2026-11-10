"""PHASE-D: PHASE with vehicle dynamics ("growing balls", priority braking,
deflections that run to completion).

Experimental variant of PHASE V1.2, built on the same pieces: the PHASE
``Vehicle``, ``FramedAStarEngine``, ``calculate_90_deg_deflection`` and the
momentum-weighted deflection distance, plus V1.2's PBD pass. The PHASE source
file is not modified. What changes:

1. Growing balls. Each vehicle's collision ball grows with its own speed
   *towards* the other object:
       R_i = margin_i + c_i * t_react + c_i^2 / (2 * a_brake)
   with c_i = the vehicle's velocity component along the line to the other
   object (0 if it isn't approaching). Head-on, the balls grow by the full
   stopping distance; parallel or diverging traffic keeps the bare margin.
   A conflict starts when two balls touch (d < R_i + R_j + buffer).

2. Priority braking. Every tick, for every pair, the closing speed of the
   vehicles' desired velocities is capped so the gap can still be closed
   with braking (c*t_react + c^2/(2*a_brake) <= gap). The excess is removed
   from the LOWER-momentum vehicle first (it gives up its approach velocity
   and may back away up to its own speed); only what it cannot remove is
   taken from the higher-momentum vehicle. Static obstacles are objects of
   infinite momentum: they never yield. Momentum ties use PHASE's own 1.5x
   tie-break, so precedence is always defined.

3. Accelerations are bounded. Commanded velocity changes by at most
   a_cmd * dt per tick, so every command is flyable. The PBD pass is kept as
   a last-resort geometric correction, but its result is also bounded;
   anything it asks for beyond that is counted as "pbd_unrealizable".

4. Deflections run to completion. A deflecting vehicle keeps flying to its
   deflection target until it gets there, stops making progress, or runs
   out of time. New conflicts no longer trigger the V1.2 global replan that
   reset every vehicle to NORMAL.

5. Subsequent deflections follow the same rule. Whenever a deflection is
   (re)started, whether for a new conflict, a new conflict while already
   deflecting, or a finished deflection whose conflict is still active,
   it uses calculate_90_deg_deflection plus the momentum-weighted distance
   max(share * speed * 4, margin * 1.5), exactly like the first one.
"""
from __future__ import annotations

import collections

import numpy as np

from phase_stepper import SAFETY_BUFFER, make_stepper_class
from spatial_grid import SpatialGrid

A_BRAKE = 2.5    # m/s^2 assumed for stopping-distance (PX4 MPC_ACC_HOR = 3.0)
A_CMD = 3.0      # m/s^2 max commanded velocity change (PX4 MPC_ACC_HOR)
T_LATENCY = 0.25          # s: one PHASE tick (0.2) + telemetry/MAVLink delay (~0.05)
TAU_RESPONSE = 1.0 / 1.8  # s: vehicle response time constant; PX4 velocity loop = 1/MPC_XY_VEL_P_ACC
T_REACT = T_LATENCY + TAU_RESPONSE
DEFLECT_STALL_S = 1.0  # deflection ends if no progress for this long


def stopping_extent(c, a=A_BRAKE, t=T_REACT):
    """Distance covered while reacting then braking from approach speed c."""
    c = max(c, 0.0)
    return c * t + c * c / (2.0 * a)


def safe_closing_speed(gap, a=A_BRAKE, t=T_REACT):
    """Largest c with stopping_extent(c) <= gap (one vehicle braking)."""
    if gap <= 0:
        return 0.0
    return a * (-t + np.sqrt(t * t + 2.0 * gap / a))


def prioritized_clip(vec, b_hat, limit):
    """Scale ``vec`` to |vec| <= limit, keeping its component along b_hat first."""
    n = float(np.linalg.norm(vec))
    if n <= limit:
        return vec
    if b_hat is None:
        return vec * (limit / n)
    along = float(np.dot(vec, b_hat))
    a_b = float(np.clip(along, -limit, limit))
    rest = vec - along * b_hat
    budget = np.sqrt(max(limit ** 2 - a_b ** 2, 0.0))
    nr = float(np.linalg.norm(rest))
    if nr > budget:
        rest = rest * (budget / nr)
    return a_b * b_hat + rest


def effective_momenta(vi, vj):
    """PHASE's momenta with its symmetric-deadlock tie-break (vi boosted 1.5x)."""
    mi, mj = vi.momentum, vj.momentum
    if mi == mj:
        mi *= 1.5
    return mi, mj


def make_dynamic_stepper_class(mod, a_brake=A_BRAKE, a_cmd=A_CMD, t_latency=T_LATENCY,
                               tau_response=TAU_RESPONSE, use_grid=True):
    """``tau_response``: how long the vehicle takes to follow a velocity change
    (first-order time constant). It sizes the balls (reaction distance
    c * (t_latency + tau)) and, in braking, the extra distance a vehicle covers
    while its real velocity catches up with the command: (v_meas - v_cmd) * tau.
    ``use_grid``: spatial hash for neighbour queries (False = O(N^2) reference)."""
    Base = make_stepper_class(mod)

    class DynamicPhaseStepper(Base):
        variant = "phase_d"

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.a_brake, self.a_cmd = a_brake, a_cmd
            self.t_latency, self.tau = t_latency, tau_response
            self.t_react = t_latency + tau_response
            self.use_grid = use_grid
            self.counters = collections.Counter()
            self._grid = None      # all vehicles, current positions, built each tick
            self._index = {}

        def step(self, measured=None, quiet=True, measured_vel=None):
            # Measured velocity only sizes the balls (meas_vel). Commands ramp from
            # the previous *commanded* velocity (vel): anchoring them to the
            # measured velocity left PX4 a tiny velocity error, so it braked at
            # ~1 m/s^2 instead of the 2.5-3 m/s^2 the balls assume.
            if measured_vel:
                self._closed_loop = True
                for v in self.vehicles:
                    if v.v_id in measured_vel:
                        v.meas_vel = np.array(measured_vel[v.v_id], dtype=float)
            return super().step(measured, quiet=quiet)

        def add_vehicle(self, v):
            super().add_vehicle(v)
            v.vel = np.zeros(3)        # commanded-velocity state the ramp starts from
            v.meas_vel = np.zeros(3)   # measured velocity (= vel in kinematic runs)
            v.cmd_vel = np.zeros(3)    # velocity commanded this tick
            v.des_vel = np.zeros(3)
            v.defl_started = None
            v.defl_best = np.inf
            v.defl_best_t = 0.0
            v.defl_deadline = np.inf
            v.n_deflections = 0

        # -------------------------------------------------------------- helpers
        def _flat(self, x):
            x = np.array(x, dtype=float)
            if not self.allow_3d:
                x[2] = 0.0
            return x

        def _goal(self, v):
            g = np.copy(v.end)
            if not self.allow_3d:
                g[2] = v.locked_altitude
            return g

        def _nearby(self, v, radius):
            """Active, not-yet-arrived vehicles within ``radius`` of v."""
            if self.use_grid and self._grid is not None:
                cands = [self.vehicles[j] for j in self._grid.neighbors(self._index[v.v_id], radius)]
            else:
                cands = [o for o in self.vehicles if o is not v
                         and np.linalg.norm(self._flat(o.current_pos - v.current_pos)) <= radius]
            return [o for o in cands if o.is_active and not o.reached_goal]

        def _deflection_direction(self, v, partner):
            """PHASE's 90-degree deflection, scored against every nearby vehicle.

            Same structure as calculate_90_deg_deflection: candidates are 90 deg
            left/right of the current heading, scored 3 x evasion + 1 x goal, with
            the same obstacle repulsion. The evasion direction now sums a push
            away from *every* vehicle inside the planning frame (astar_frame_size),
            each weighted by how close it is relative to the distance at which
            its ball and v's would touch. With only the partner nearby and no
            obstacles this gives exactly PHASE's original choice."""
            dir_i = v.last_dir
            left_90 = np.array([-dir_i[1], dir_i[0], 0.0])
            right_90 = np.array([dir_i[1], -dir_i[0], 0.0])
            others = self._nearby(v, self.astar_frame_size)
            if partner not in others:
                others.append(partner)
            evasion = np.zeros(3)
            for o in others:
                away = self._flat(v.current_pos - o.current_pos)
                d = float(np.linalg.norm(away))
                if d < 1e-6:
                    continue
                u = -away / d  # from v towards o
                reach = (v.margin + o.margin + SAFETY_BUFFER
                         + stopping_extent(self._approach(v, u), self.a_brake, self.t_react)
                         + stopping_extent(self._approach(o, -u), self.a_brake, self.t_react))
                evasion += reach * (away / d)   # partner alone -> same direction as PHASE's
            for obs in self.obstacles:
                if obs.z_min <= v.current_pos[2] <= obs.z_max:
                    vec2d = v.current_pos[:2] - np.array([obs.x, obs.y])
                    dist2d = np.linalg.norm(vec2d)
                    if dist2d < (obs.radius + v.margin + 5.0):
                        repel = vec2d / dist2d if dist2d > 1e-3 else np.array([1.0, 0.0])
                        evasion[:2] += repel * ((obs.radius + v.margin + 5.0) - dist2d) * 2.0
            n_ev = np.linalg.norm(evasion)
            evasion = evasion / n_ev if n_ev > 1e-3 else np.array([1.0, 0.0, 0.0])
            goal_dir = v.end - v.current_pos
            goal_dir[2] = 0
            n_goal = np.linalg.norm(goal_dir)
            goal_dir = goal_dir / n_goal if n_goal > 1e-3 else dir_i
            score_left = 3.0 * np.dot(left_90, evasion) + 1.0 * np.dot(left_90, goal_dir)
            score_right = 3.0 * np.dot(right_90, evasion) + 1.0 * np.dot(right_90, goal_dir)
            return (left_90 if score_left > score_right else right_90), [o.v_id for o in others]

        def _deflect(self, v, other, mom_v, mom_o, kind):
            """PHASE's deflection rule: 90-degree evasion, momentum-weighted distance."""
            heading = np.copy(v.last_dir)
            direction, considered = self._deflection_direction(v, other)
            share = mom_o / (mom_v + mom_o)
            dist = max(share * (v.speed * 4.0), v.margin * 1.5)
            v.deflection_target = v.current_pos + direction * dist
            if not self.allow_3d:
                v.deflection_target[2] = v.locked_altitude
            v.state = 'DEFLECTING'
            v.waypoints.clear()
            v.defl_started = self.t
            v.defl_best = dist
            v.defl_best_t = self.t
            v.defl_deadline = self.t + dist / max(v.speed, 1e-3) + 3.0
            v.n_deflections += 1
            v.last_dir = direction
            self.counters[f"deflections_{kind}"] += 1
            self.deflection_events.append({'v_id': v.v_id, 'pos': np.copy(v.current_pos), 'kind': kind,
                                           'partner': other.v_id, 't': round(self.t, 2),
                                           'heading': heading, 'direction': np.copy(direction),
                                           'dist': dist, 'share': share, 'considered': considered})

        def _approach(self, v, toward_unit):
            """Approach speed of v along toward_unit: the largest of measured, commanded and desired."""
            return max(0.0, float(np.dot(v.meas_vel, toward_unit)), float(np.dot(v.vel, toward_unit)),
                       float(np.dot(v.des_vel, toward_unit)))

        def _balls_touch(self, vi, vj):
            d_vec = self._flat(vj.current_pos - vi.current_pos)
            d = float(np.linalg.norm(d_vec))
            if d < 1e-6:
                return True, d
            u = d_vec / d
            ri = vi.margin + stopping_extent(self._approach(vi, u), self.a_brake, self.t_react)
            rj = vj.margin + stopping_extent(self._approach(vj, -u), self.a_brake, self.t_react)
            return d < ri + rj + SAFETY_BUFFER, d

        # ----------------------------------------------------------------- tick
        def _tick(self):
            self.t += self.dt
            t = self.t
            dt = self.dt
            astar_calls = 0
            prev_conflicts = self.active_conflict_events
            conflict_partners = collections.defaultdict(set)
            for a_id, b_id in prev_conflicts:
                conflict_partners[a_id].add(b_id)
                conflict_partners[b_id].add(a_id)
            by_id = {v.v_id: v for v in self.vehicles}

            # Neighbour index for this tick (positions don't move until the commit).
            self._index = {v.v_id: k for k, v in enumerate(self.vehicles)}
            vmax = max([max(v.speed, float(np.linalg.norm(v.meas_vel)), float(np.linalg.norm(v.vel)))
                        for v in self.vehicles] or [0.0])
            mmax = max([v.margin for v in self.vehicles] or [0.0])
            # Farthest two vehicles can be apart and still interact this tick:
            # touching balls, or braking at closing speed 2*vmax incl. response lag.
            pair_radius = (2 * mmax + SAFETY_BUFFER + stopping_extent(2 * vmax, self.a_brake, self.t_react)
                           + 2 * vmax * self.tau + 1.0)
            if self.use_grid:
                self._grid = SpatialGrid(max(pair_radius, self.astar_frame_size), use_z=self.allow_3d)
                self._grid.build([v.current_pos for v in self.vehicles])

            # PHASE 1: state, deflection completion, desired velocity
            for v in self.vehicles:
                if not v.is_active:
                    if t >= v.start_time:
                        v.is_active = True
                    else:
                        v.prop_pos = np.copy(v.current_pos)
                        continue
                if not self.allow_3d:
                    v.current_pos[2] = v.locked_altitude
                    v.last_pos[2] = v.locked_altitude
                goal = self._goal(v)
                dist_to_goal = float(np.linalg.norm(goal - v.current_pos))
                if dist_to_goal < 0.5:
                    v.reached_goal = True
                    v.state = 'NORMAL'
                    v.des_vel = np.zeros(3)
                    continue
                v.reached_goal = False

                if v.state == 'DEFLECTING':
                    to_t = self._flat(v.deflection_target - v.current_pos)
                    dist_t = float(np.linalg.norm(to_t))
                    if dist_t < v.defl_best - 0.05:
                        v.defl_best, v.defl_best_t = dist_t, t
                    arrived = dist_t < max(0.5, float(np.linalg.norm(v.vel)) * dt)
                    stalled = t - v.defl_best_t > DEFLECT_STALL_S
                    expired = t > v.defl_deadline
                    if arrived or stalled or expired:
                        self.counters["deflections_finished_" + ("arrived" if arrived else "stalled" if stalled else "expired")] += 1
                        # Still in conflict with someone? Deflect again by the same rule.
                        partners = [by_id[p] for p in sorted(conflict_partners[v.v_id])
                                    if by_id[p].is_active and not by_id[p].reached_goal]
                        if partners:
                            o = min(partners, key=lambda o: np.linalg.norm(o.current_pos - v.current_pos))
                            mv, mo = effective_momenta(v, o)
                            self._deflect(v, o, mv, mo, "subsequent")
                        else:
                            v.state = 'NORMAL'
                            v.deflection_target = None
                            v.needs_replan = True
                    if v.state == 'DEFLECTING':
                        to_t = self._flat(v.deflection_target - v.current_pos)
                        n = float(np.linalg.norm(to_t))
                        v.des_vel = to_t / n * v.speed if n > 1e-6 else np.zeros(3)
                        continue

                # NORMAL: follow the A* path (replanned when flagged)
                if v.needs_replan:
                    walls = [(np.copy(o.current_pos), o.margin) for o in self._nearby(v, self.astar_frame_size)]
                    path = self.astar.compute_path(v, v.current_pos, v.end, self.obstacles, walls)
                    astar_calls += 1
                    if not self.allow_3d:
                        path = [np.array([p[0], p[1], v.locked_altitude], dtype=float) for p in path]
                    v.waypoints = collections.deque(path)
                    v.needs_replan = False
                # Advance past waypoints that are reached OR already passed (the vehicle is
                # beyond the waypoint on the way to the next one). Without the "passed" test
                # an acceleration-limited vehicle can orbit a waypoint that falls inside its
                # turning circle (v^2 / a_cmd) forever (seen on PX4, CHANGELOG D-11).
                pop_r = max(0.5, float(np.linalg.norm(v.vel)) * dt)
                while len(v.waypoints) > 1:
                    wp, nxt = v.waypoints[0], v.waypoints[1]
                    reached = np.linalg.norm(self._flat(wp - v.current_pos)) < pop_r
                    passed = float(np.dot(self._flat(v.current_pos - wp), self._flat(nxt - wp))) > 0.0
                    if not (reached or passed):
                        break
                    v.waypoints.popleft()
                    self.counters["waypoints_passed" if passed and not reached else "waypoints_reached"] += 1
                target = v.waypoints[0] if v.waypoints else goal
                to_t = self._flat(target - v.current_pos)
                n = float(np.linalg.norm(to_t))
                # Arrive at the goal without overshooting: cap speed by the braking curve.
                speed = min(v.speed, safe_closing_speed(dist_to_goal, self.a_cmd, 0.0) if dist_to_goal > 0 else 0.0)
                if n > 1e-6:
                    v.des_vel = to_t / n * speed
                    v.last_dir = to_t / n
                else:
                    v.des_vel = np.zeros(3)

            active = [v for v in self.vehicles if v.is_active and not v.reached_goal]
            n_act = len(active)
            # Candidate pairs: grid query, O(N*k); or every pair, O(N^2).
            if self.use_grid:
                pairs = SpatialGrid(pair_radius, use_z=self.allow_3d).build(
                    [v.current_pos for v in active]).pairs(pair_radius)
            else:
                pairs = [(i, j) for i in range(n_act) for j in range(i + 1, n_act)]
            self.counters["pair_checks"] += len(pairs)

            # PHASE 2: growing-ball conflicts -> deflection rule (first or subsequent)
            current_conflicts = set()
            new_pairs = []
            for i, j in pairs:
                vi, vj = active[i], active[j]
                touch, _ = self._balls_touch(vi, vj)
                if not touch:
                    continue
                pair = tuple(sorted((vi.v_id, vj.v_id)))
                current_conflicts.add(pair)
                if pair in prev_conflicts:
                    continue
                new_pairs.append((vi, vj))
            # One deflection per vehicle per tick, against its nearest new partner,
            # so the outcome doesn't depend on pair order.
            nearest = {}
            for vi, vj in new_pairs:
                mi, mj = effective_momenta(vi, vj)
                if vi.momentum == vj.momentum:
                    self.identified_deadlocks.add(tuple(sorted((vi.v_id, vj.v_id))))
                d = float(np.linalg.norm(vi.current_pos - vj.current_pos))
                for v, o, mv, mo in ((vi, vj, mi, mj), (vj, vi, mj, mi)):
                    if v.v_id not in nearest or d < nearest[v.v_id][0]:
                        nearest[v.v_id] = (d, v, o, mv, mo)
            for _, v, o, mv, mo in nearest.values():
                kind = "subsequent" if v.state == 'DEFLECTING' or v.n_deflections else "initial"
                self._deflect(v, o, mv, mo, kind)
                to_t = self._flat(v.deflection_target - v.current_pos)
                n = float(np.linalg.norm(to_t))
                v.des_vel = to_t / n * v.speed if n > 1e-6 else np.zeros(3)
            new_pairs = [tuple(sorted((a.v_id, b.v_id))) for a, b in new_pairs]
            self.active_conflict_events = current_conflicts
            if new_pairs:
                # V1.2 replanned every vehicle on any new conflict. Here deflections
                # are left to finish, and only NORMAL vehicles within the planning
                # frame of a new conflict replan; with a large fleet "any new
                # conflict anywhere" otherwise means everyone runs A* every tick.
                involved = {vid for p in new_pairs for vid in p}
                nearby_ids = set()
                for vid in involved:
                    nearby_ids.update(o.v_id for o in self._nearby(by_id[vid], self.astar_frame_size))
                for v in active:
                    if v.state == 'NORMAL' and v.v_id not in involved and v.v_id in nearby_ids:
                        v.needs_replan = True

            # PHASE 3: priority braking on the velocities that will actually be
            # commanded this tick (desired velocity after the acceleration limit).
            # Braking the *desired* velocity is not enough: right after a deflection
            # the desired velocity already points away while the vehicle is still
            # mostly on its old heading.
            dv_max = self.a_cmd * dt
            cand = {}
            for v in active:
                v.des_vel = prioritized_clip(self._flat(v.des_vel), None, v.speed)
                cand[v.v_id] = self._flat(v.vel + prioritized_clip(self._flat(v.des_vel - v.vel), None, dv_max))
            cand_raw = {k: np.copy(c) for k, c in cand.items()}
            brake_ticks = 0
            residual = 0.0
            for _ in range(3):
                for i, j in pairs:
                    vi, vj = active[i], active[j]
                    d_vec = self._flat(vj.current_pos - vi.current_pos)
                    d = float(np.linalg.norm(d_vec))
                    if d < 1e-6:
                        continue
                    u = d_vec / d
                    gap = d - (vi.margin + vj.margin + SAFETY_BUFFER)
                    closing = float(np.dot(cand[vi.v_id] - cand[vj.v_id], u))
                    # Response lag: while the real velocities catch up with the
                    # commands, the pair closes an extra (v_meas - v_cmd) * tau.
                    meas_closing = float(np.dot(vi.meas_vel - vj.meas_vel, u))
                    gap -= max(0.0, meas_closing - closing) * self.tau
                    excess = closing - safe_closing_speed(gap, self.a_brake, self.t_react)
                    if excess <= 1e-9:
                        continue
                    brake_ticks += 1
                    mi, mj = effective_momenta(vi, vj)
                    # (vehicle, sign of its approach along u): vi approaches along +u, vj along -u
                    order = [(vi, 1.0), (vj, -1.0)] if mi < mj else [(vj, -1.0), (vi, 1.0)]
                    for v, s in order:
                        if excess <= 1e-9:
                            break
                        comp = s * float(np.dot(cand[v.v_id], u))  # its approach speed (may be negative)
                        room = comp + v.speed                      # can back away down to -speed
                        take = min(excess, max(room, 0.0))
                        cand[v.v_id] = cand[v.v_id] - s * take * u
                        excess -= take
                    residual = max(residual, excess)
                # Obstacles: infinite momentum, the vehicle always yields.
                for v in active:
                    for obs in self.obstacles:
                        if not (obs.z_min <= v.current_pos[2] <= obs.z_max):
                            continue
                        vec = np.array([obs.x - v.current_pos[0], obs.y - v.current_pos[1], 0.0])
                        d = float(np.linalg.norm(vec))
                        if d < 1e-6:
                            continue
                        u = vec / d
                        gap = d - (obs.radius + v.margin + SAFETY_BUFFER)
                        comp = float(np.dot(cand[v.v_id], u))
                        gap -= max(0.0, float(np.dot(v.meas_vel, u)) - comp) * self.tau
                        excess = comp - safe_closing_speed(gap, self.a_brake, self.t_react)
                        if excess > 1e-9:
                            brake_ticks += 1
                            cand[v.v_id] = cand[v.v_id] - min(excess, comp + v.speed) * u

            # Commit the commands within the speed and acceleration limits; braking
            # gets first call on the budget, steering (path / deflection turn) the rest.
            for v in active:
                brake = cand[v.v_id] - cand_raw[v.v_id]
                nb = float(np.linalg.norm(brake))
                b_hat = brake / nb if nb > 1e-6 else None
                dv = prioritized_clip(self._flat(cand[v.v_id] - v.vel), b_hat, dv_max)
                v.cmd_vel = prioritized_clip(self._flat(v.vel + dv), b_hat, v.speed)
                v.brake_dir = b_hat
                v.prop_pos = v.current_pos + v.cmd_vel * dt
                if not self.allow_3d:
                    v.prop_pos[2] = v.locked_altitude
            for v in self.vehicles:
                if v.is_active and v.reached_goal:
                    # Brake to a hover on the goal.
                    dv = self._flat(-v.vel)
                    n = float(np.linalg.norm(dv))
                    if n > dv_max:
                        dv *= dv_max / n
                    v.cmd_vel = self._flat(v.vel + dv)
                    v.prop_pos = v.current_pos + v.cmd_vel * dt
                    if not self.allow_3d:
                        v.prop_pos[2] = v.locked_altitude

            # PHASE 4: V1.2's PBD as a last resort, then bound what it asks for
            pre = {v.v_id: np.copy(v.prop_pos) for v in active}
            pbd_iters = 0
            for _ in range(1000):
                pbd_iters += 1
                for v in active:
                    for obs in self.obstacles:
                        if obs.z_min <= v.prop_pos[2] <= obs.z_max:
                            vec2d = v.prop_pos[:2] - np.array([obs.x, obs.y])
                            dist2d = np.linalg.norm(vec2d)
                            req = obs.radius + v.margin + SAFETY_BUFFER
                            if dist2d < req:
                                v.prop_pos[:2] += (vec2d / dist2d if dist2d > 1e-5 else np.array([1.0, 0.0])) * (req - dist2d)
                clear = True
                for i, j in pairs:
                    vi, vj = active[i], active[j]
                    req = vi.margin + vj.margin + SAFETY_BUFFER
                    delta = vi.prop_pos - vj.prop_pos
                    dist = np.linalg.norm(delta)
                    if dist < req:
                        clear = False
                        direction = delta / dist if dist > 1e-5 else np.array([1.0, 0.0, 0.0])
                        if not self.allow_3d:
                            direction[2] = 0.0
                        mi, mj = effective_momenta(vi, vj)
                        vi.prop_pos += direction * (req - dist) * (mj / (mi + mj)) * 0.8
                        vj.prop_pos -= direction * (req - dist) * (mi / (mi + mj)) * 0.8
                if not self.allow_3d:
                    for v in active:
                        v.prop_pos[2] = v.locked_altitude
                if clear:
                    break
            pbd_shift = {}
            unrealizable = 0.0
            for v in active:
                shift = float(np.linalg.norm(v.prop_pos - pre[v.v_id]))
                pbd_shift[v.v_id] = shift
                if shift > 1e-6:
                    want = self._flat((v.prop_pos - v.current_pos) / dt)
                    dv = want - v.vel
                    n = float(np.linalg.norm(dv))
                    if n > dv_max:
                        unrealizable = max(unrealizable, (n - dv_max) * dt)
                        dv *= dv_max / n
                    v.cmd_vel = prioritized_clip(self._flat(v.vel + dv), None, v.speed)
                    v.prop_pos = v.current_pos + v.cmd_vel * dt
                    if not self.allow_3d:
                        v.prop_pos[2] = v.locked_altitude
            if any(s > 1e-6 for s in pbd_shift.values()):
                self.counters["pbd_ticks"] += 1
            if unrealizable > 0:
                self.counters["pbd_unrealizable_ticks"] += 1
            if brake_ticks:
                self.counters["brake_ticks"] += 1

            # PHASE 5: commit (as V1.2), plus the velocity state
            for v in self.vehicles:
                if not v.is_active:
                    continue
                v.last_pos = np.copy(v.current_pos)
                v.current_pos = np.copy(v.prop_pos)
                v.vel = np.copy(v.cmd_vel)   # the next command ramps from this one
                if not getattr(self, "_closed_loop", False):
                    v.meas_vel = np.copy(v.cmd_vel)  # kinematic: perfect tracking
                if not self.allow_3d:
                    v.current_pos[2] = v.locked_altitude
                v.resolved_path.append({'t': t, 'pos': np.copy(v.current_pos)})
                v.state_history.append(v.state)
                if np.linalg.norm(v.current_pos - self._goal(v)) < 0.5:
                    v.reached_goal = True
                    if v.new_time == 0.0:
                        v.new_time = t
                else:
                    if np.linalg.norm(v.current_pos - v.last_progress_pos) < 0.1:
                        v.stuck_time += dt
                        if v.stuck_time > 3.0 and not v.deadlock_reported:
                            print(f"🚨 POSITIONAL DEADLOCK IDENTIFIED: Veh {v.v_id} is stuck at {np.round(v.current_pos, 2)}.")
                            v.deadlock_reported = True
                    else:
                        v.deadlock_reported = False
                        v.last_progress_pos = np.copy(v.current_pos)
                        v.stuck_time = 0.0

            return {
                't': t,
                'new_conflicts': new_pairs,
                'active_conflicts': sorted(current_conflicts),
                'pbd_iters': pbd_iters,
                'pbd_shift': pbd_shift,
                'pbd_unrealizable_m': unrealizable,
                'brake_pairs': brake_ticks,
                'brake_residual_mps': residual,
                'astar_calls': astar_calls,
                'states': {v.v_id: v.state for v in self.vehicles},
            }

    return DynamicPhaseStepper
