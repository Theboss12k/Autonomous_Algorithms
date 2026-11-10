"""Rules of the PHASE-D variant (growing balls, priority braking, deflections)."""
import itertools

import numpy as np
import pytest

import scenarios
from phase_dynamic import A_CMD, T_REACT, A_BRAKE, make_dynamic_stepper_class, safe_closing_speed, stopping_extent
from phase_stepper import SAFETY_BUFFER, load_phase

MOD = load_phase()
Stepper = make_dynamic_stepper_class(MOD)


def run_kinematic(name, max_t=150.0):
    sim = scenarios.build(MOD, Stepper, name)
    sim.build_original_paths()
    while sim.t < max_t and not sim.all_reached():
        sim.step()
    return sim


def two(prio_a, prio_b, pos_a, pos_b, vel_a, vel_b, already_in_conflict=True):
    sim = Stepper(astar_frame_size=15.0, allow_3d=False)
    a = MOD.Vehicle("A", pos_a, [pos_a[0] + 100, pos_a[1], 10], 4.0, prio_a, 2.5)
    b = MOD.Vehicle("B", pos_b, [pos_b[0] - 100, pos_b[1], 10], 4.0, prio_b, 2.5)
    sim.add_vehicle(a)
    sim.add_vehicle(b)
    a.vel, b.vel = np.array(vel_a, float), np.array(vel_b, float)
    if already_in_conflict:  # isolate braking from the deflection trigger
        sim.active_conflict_events = {("A", "B")}
    return sim, a, b


def test_safe_closing_speed_inverts_stopping_extent():
    for gap in [0.5, 2.0, 7.3, 20.0]:
        assert stopping_extent(safe_closing_speed(gap)) == pytest.approx(gap)


def test_ball_grows_head_on_but_not_for_parallel_traffic():
    # Head-on at 4 m/s each, 14 m apart: bare margins (5.2 m) are far apart, grown balls touch.
    sim, a, b = two(1, 1, [0, 0, 10], [14, 0, 10], [4, 0, 0], [-4, 0, 0], already_in_conflict=False)
    a.des_vel, b.des_vel = a.vel.copy(), b.vel.copy()
    assert sim._balls_touch(a, b)[0]
    expected = 2 * (2.5 + stopping_extent(4.0)) + SAFETY_BUFFER
    assert expected > 14 > 2.5 + 2.5 + SAFETY_BUFFER
    # Same velocity side by side, 6 m apart: no approach, balls stay at the bare margin.
    sim, a, b = two(1, 1, [0, 0, 10], [0, 6, 10], [4, 0, 0], [4, 0, 0], already_in_conflict=False)
    a.des_vel, b.des_vel = a.vel.copy(), b.vel.copy()
    assert not sim._balls_touch(a, b)[0]


def test_lower_priority_brakes_first_higher_priority_holds_course():
    # A (priority 10) and B (priority 1) closing head-on inside braking range.
    sim, a, b = two(10, 1, [0, 0, 10], [8, 0, 10], [4, 0, 0], [-4, 0, 0])
    sim.step()
    assert a.cmd_vel[0] == pytest.approx(4.0, abs=1e-6)          # high priority keeps its speed
    assert -b.cmd_vel[0] < 4.0 - 0.05                             # low priority's approach speed drops
    assert np.linalg.norm(b.cmd_vel - np.array([-4.0, 0, 0])) <= A_CMD * sim.dt + 1e-9  # within accel limit


def test_higher_priority_brakes_only_for_what_the_lower_cannot_remove():
    # B (low priority) is hovering, A (high priority) bears down on it: B backs away, and
    # A only brakes if B's room (backing away at up to its speed) isn't enough.
    sim, a, b = two(10, 1, [0, 0, 10], [6.0, 0, 10], [4, 0, 0], [0, 0, 0])
    sim.step()
    assert b.cmd_vel[0] > 0                                       # B moves away from A
    gap = 6.0 - (2.5 + 2.5 + SAFETY_BUFFER)
    need = 4.0 - safe_closing_speed(gap)
    if need <= 4.0:                                               # B alone can absorb it
        assert a.cmd_vel[0] == pytest.approx(4.0, abs=1e-6)


def test_obstacles_never_yield():
    sim = Stepper(astar_frame_size=15.0, allow_3d=False)
    sim.add_obstacle(MOD.CylinderObstacle(x=10, y=0, radius=3, z_min=0, z_max=30))
    v = MOD.Vehicle("A", [0, 0, 10], [30, 0, 10], 4.0, 1000.0, 2.5)
    sim.add_vehicle(v)
    v.vel = np.array([4.0, 0, 0])
    sim.step()
    assert v.cmd_vel[0] < 4.0  # even a very high-priority vehicle brakes for a static object


@pytest.mark.parametrize("name", list(scenarios.SCENARIOS))
def test_kinematic_scenarios_complete_safely_within_limits(name):
    sim = run_kinematic(name)
    assert sim.all_reached(), {v.v_id: v.reached_goal for v in sim.vehicles}
    paths = {v.v_id: {round(p['t'], 2): p['pos'] for p in v.resolved_path} for v in sim.vehicles}
    done = {v.v_id: v.new_time for v in sim.vehicles}
    for a, b in itertools.combinations(sim.vehicles, 2):
        ts = [t for t in paths[a.v_id] if 0 < t <= min(done[a.v_id], done[b.v_id]) and t in paths[b.v_id]]
        sep = min(np.linalg.norm(paths[a.v_id][t][:2] - paths[b.v_id][t][:2]) for t in ts)
        assert sep >= a.margin + b.margin - 0.05, (a.v_id, b.v_id, sep)
    for v in sim.vehicles:
        pts = np.array([p['pos'] for p in v.resolved_path])
        vel = np.diff(pts, axis=0) / sim.dt
        assert np.linalg.norm(vel, axis=1).max() <= v.speed + 1e-6
        acc = np.diff(vel, axis=0) / sim.dt
        assert np.linalg.norm(acc, axis=1).max() <= A_CMD + 1e-6


@pytest.mark.parametrize("name", list(scenarios.SCENARIOS))
def test_every_deflection_follows_the_rule(name):
    sim = run_kinematic(name)
    assert sim.deflection_events
    for e in sim.deflection_events:
        v = next(x for x in sim.vehicles if x.v_id == e['v_id'])
        o = next(x for x in sim.vehicles if x.v_id == e['partner'])
        # 90 degrees off the heading at the time, in the horizontal plane
        assert abs(np.dot(e['heading'][:2], e['direction'][:2])) < 1e-6
        assert np.linalg.norm(e['direction']) == pytest.approx(1.0)
        # momentum-weighted distance, with PHASE's 1.5x tie-break
        mv, mo = v.momentum, o.momentum
        shares = {mo / (mv + mo)} if mv != mo else {mo / (1.5 * mv + mo), 1.5 * mo / (mv + 1.5 * mo)}
        assert any(abs(e['share'] - s) < 1e-9 for s in shares)
        assert e['dist'] == pytest.approx(max(e['share'] * v.speed * 4.0, v.margin * 1.5))


def test_subsequent_deflections_happen_and_deflections_are_not_reset():
    sim = run_kinematic("head_on_cross")
    kinds = {e['kind'] for e in sim.deflection_events}
    assert {"initial", "subsequent"} <= kinds
    # A deflection lasts several ticks (V1.2 reset it to NORMAL in the same tick).
    for v in sim.vehicles:
        runs, cur = [], 0
        for s in v.state_history:
            cur = cur + 1 if s == 'DEFLECTING' else 0
            runs.append(cur)
        assert max(runs) >= 3, v.v_id


def test_no_orbiting_a_waypoint_inside_the_turning_circle():
    """Regression for D-11: Q2 orbited an A* waypoint ~20 s on PX4. A vehicle moving at
    full speed past a waypoint 2 m to its side (inside its 5.3 m turning circle) must
    move on to the next waypoint instead of circling it."""
    sim = Stepper(astar_frame_size=15.0, allow_3d=False)
    v = MOD.Vehicle("A", [0, 0, 10], [30, 0, 10], 4.0, 1.0, 2.5)
    sim.add_vehicle(v)
    sim.step()                                    # activates and plans
    import collections as _c
    v.waypoints = _c.deque([np.array([1.0, 2.0, 10.0]), np.array([30.0, 0.0, 10.0])])
    v.vel = np.array([4.0, 0.0, 0.0])
    v.needs_replan = False
    xs = []
    for _ in range(40):
        sim.step()
        xs.append(v.current_pos[0])
    assert xs[-1] > 20.0                          # made progress to the goal, no orbit
    assert np.all(np.diff(xs) > -1e-6)            # never turned back


def test_three_way_swap_no_longer_livelocks():
    sim = run_kinematic("three_way_swap")
    assert sim.all_reached()


@pytest.mark.parametrize("name", list(scenarios.SCENARIOS))
def test_separation_holds_with_px4_like_tracking_lag(name):
    """Closed loop against a PX4-like lagging controller (lag_plant): the case that
    exposed braking anchored to measured velocity, and braking on desired velocity."""
    import lag_plant
    sim = scenarios.build(MOD, Stepper, name)
    trace = lag_plant.fly(sim)
    assert sim.all_reached()
    need = min(v.margin for v in sim.vehicles) * 2
    assert lag_plant.min_separation(sim, trace) >= need - 0.05
