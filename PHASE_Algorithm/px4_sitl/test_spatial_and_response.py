"""Spatial grid (O(N*k) neighbour queries), multi-vehicle deflection scoring (1a),
and the response-time parameter (1b)."""
import numpy as np
import pytest

import scenarios
from phase_dynamic import SAFETY_BUFFER, make_dynamic_stepper_class, safe_closing_speed
from phase_stepper import load_phase
from spatial_grid import SpatialGrid, brute_pairs

MOD = load_phase()
Grid = make_dynamic_stepper_class(MOD, use_grid=True)
Brute = make_dynamic_stepper_class(MOD, use_grid=False)


# ---------------------------------------------------------------- spatial grid
@pytest.mark.parametrize("seed", range(5))
@pytest.mark.parametrize("radius", [3.0, 15.0, 40.0])
def test_grid_pairs_equal_brute_force(seed, radius):
    pts = np.random.default_rng(seed).uniform(0, 200, (300, 3))
    pts[:, 2] = 10.0
    assert SpatialGrid(radius).build(pts).pairs(radius) == brute_pairs(pts, radius)


def test_grid_neighbors_equal_brute_force():
    pts = np.random.default_rng(7).uniform(0, 100, (200, 3))
    g = SpatialGrid(12.0).build(pts)
    for i in range(0, 200, 17):
        want = [j for j in range(200) if j != i and np.hypot(*(pts[i, :2] - pts[j, :2])) <= 12.0]
        assert g.neighbors(i, 12.0) == want


def run(cls, build, max_t=120.0):
    sim = build(cls)
    sim.build_original_paths()
    while sim.t < max_t and not sim.all_reached():
        sim.step()
    return sim


@pytest.mark.parametrize("which", list(scenarios.SCENARIOS) + ["random30"])
def test_phase_d_with_grid_is_identical_to_brute_force(which):
    if which == "random30":
        build = lambda cls: scenarios.build_random(MOD, cls, 30, seed=3)  # noqa: E731
    else:
        build = lambda cls: scenarios.build(MOD, cls, which)  # noqa: E731
    g, b = run(Grid, build, 60.0), run(Brute, build, 60.0)
    for vg, vb in zip(g.vehicles, b.vehicles):
        pg = np.array([p['pos'] for p in vg.resolved_path])
        pb = np.array([p['pos'] for p in vb.resolved_path])
        np.testing.assert_allclose(pg, pb, atol=1e-9, err_msg=vg.v_id)
    assert g.counters["pair_checks"] <= b.counters["pair_checks"]


# ------------------------------------------------- 1a: deflection vs all nearby
def fleet(positions, priorities=None, heading=(1.0, 0.0, 0.0)):
    sim = Grid(astar_frame_size=15.0, allow_3d=False)
    for k, p in enumerate(positions):
        v = MOD.Vehicle(f"V{k}", [p[0], p[1], 10], [p[0] + 50, p[1], 10], 4.0,
                        (priorities or [1.0] * len(positions))[k], 2.5)
        sim.add_vehicle(v)
        v.is_active = True
        v.last_dir = np.array(heading, float)
    sim._index = {v.v_id: k for k, v in enumerate(sim.vehicles)}
    sim._grid = SpatialGrid(30.0).build([v.current_pos for v in sim.vehicles])
    return sim


def test_partner_only_matches_phase_original_rule():
    rng = np.random.default_rng(1)
    for _ in range(50):
        p = rng.uniform(-10, 10, 2)
        sim = fleet([[0, 0], p], heading=(*(h := rng.normal(size=2)) / np.linalg.norm(h), 0.0))
        v, o = sim.vehicles
        mine, _ = sim._deflection_direction(v, o)
        np.testing.assert_allclose(mine, MOD.calculate_90_deg_deflection(v, o, []))


def test_third_vehicle_changes_the_side():
    # V0 heads +x. Partner V1 is slightly to the RIGHT ahead, so PHASE's single-partner
    # rule sends V0 LEFT (+y), straight at V2, which sits close on the left.
    sim = fleet([[0, 0], [8, -1], [2, 5]])
    v, partner, third = sim.vehicles
    original = MOD.calculate_90_deg_deflection(v, partner, [])
    assert original[1] > 0                                    # original rule: left, towards V2
    mine, considered = sim._deflection_direction(v, partner)
    assert set(considered) == {"V1", "V2"}
    assert mine[1] < 0                                        # all-nearby rule: right, away from both


def test_vehicles_outside_the_planning_frame_are_ignored():
    sim = fleet([[0, 0], [8, -1], [2, 40]])                   # V2 is 40 m away, frame is 15 m
    _, considered = sim._deflection_direction(sim.vehicles[0], sim.vehicles[1])
    assert considered == ["V1"]


# --------------------------------------------------------- 1b: response time
def test_longer_response_time_grows_the_balls():
    def touching(tau):
        cls = make_dynamic_stepper_class(MOD, tau_response=tau)
        sim = cls(astar_frame_size=15.0, allow_3d=False)
        a = MOD.Vehicle("A", [0, 0, 10], [100, 0, 10], 4.0, 1.0, 2.5)
        # 15 m apart head-on at 4 m/s: touch distance is 13.6 m with tau = 0, 21.6 m with tau = 1 s.
        b = MOD.Vehicle("B", [15, 0, 10], [-100, 0, 10], 4.0, 1.0, 2.5)
        sim.add_vehicle(a)
        sim.add_vehicle(b)
        a.vel, b.vel = np.array([4.0, 0, 0]), np.array([-4.0, 0, 0])
        return sim._balls_touch(a, b)[0]
    assert not touching(0.0)
    assert touching(1.0)


def test_lagging_vehicle_brakes_harder():
    """Same commands; when the *measured* velocity is still closing faster than the
    command (the vehicle hasn't caught up), the allowed closing speed must drop."""
    def low_priority_cmd(meas_closing_each):
        sim = Grid(astar_frame_size=15.0, allow_3d=False)
        a = MOD.Vehicle("A", [0, 0, 10], [100, 0, 10], 4.0, 10.0, 2.5)
        # 14 m apart: braking is needed but below the acceleration limit, so the lag term shows.
        b = MOD.Vehicle("B", [14, 0, 10], [-100, 0, 10], 4.0, 1.0, 2.5)
        sim.add_vehicle(a)
        sim.add_vehicle(b)
        sim.active_conflict_events = {("A", "B")}
        a.vel, b.vel = np.array([2.0, 0, 0]), np.array([-2.0, 0, 0])
        sim.step(measured_vel={"A": [meas_closing_each, 0, 0], "B": [-meas_closing_each, 0, 0]})
        return b.cmd_vel[0]
    no_lag, lagging = low_priority_cmd(2.6), low_priority_cmd(4.0)  # 2.6 = what's commanded this tick
    assert lagging > no_lag                                   # B brakes harder (less negative x)
