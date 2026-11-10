"""Three-quadcopter scenarios at quad scale.

Coordinates are PHASE's world frame, which the bridge maps to ENU:
x = East, y = North, z = Up (metres). All vehicles fly altitude-locked at
ALT, matching PHASE V1.2's 2D mode.
"""
import math

ALT = 10.0
SPEED = 4.0   # m/s, comfortable for a PX4 quad in position control
MARGIN = 2.5  # m per vehicle -> 5.2 m required centre-to-centre separation


def _circle(r, deg):
    a = math.radians(deg)
    return [round(r * math.cos(a), 3), round(r * math.sin(a), 3), ALT]


SCENARIOS = {
    # Three drones 120 deg apart on a 25 m circle swap to their antipodes, so all
    # three reach the centre together. Equal priorities exercise PHASE's
    # symmetric-deadlock momentum boost.
    "three_way_swap": {
        "astar_frame_size": 15.0,
        "obstacles": [],
        "vehicles": [
            dict(v_id="Q1", start=_circle(25, 90), end=_circle(25, 270), speed=SPEED, mass_priority=1.0, margin=MARGIN),
            dict(v_id="Q2", start=_circle(25, 210), end=_circle(25, 30), speed=SPEED, mass_priority=1.0, margin=MARGIN),
            dict(v_id="Q3", start=_circle(25, 330), end=_circle(25, 150), speed=SPEED, mass_priority=1.0, margin=MARGIN),
        ],
    },
    # Q1 (high priority) and Q2 (low priority) head-on along x while Q3 crosses
    # along y slightly later: an asymmetric two-body conflict followed by a third.
    "head_on_cross": {
        "astar_frame_size": 15.0,
        "obstacles": [],
        "vehicles": [
            dict(v_id="Q1", start=[-25, 0, ALT], end=[25, 0, ALT], speed=SPEED, mass_priority=10.0, margin=MARGIN),
            dict(v_id="Q2", start=[25, 0, ALT], end=[-25, 0, ALT], speed=SPEED, mass_priority=1.0, margin=MARGIN),
            dict(v_id="Q3", start=[0, -25, ALT], end=[0, 25, ALT], speed=3.5, mass_priority=5.0, margin=MARGIN),
        ],
    },
    # A cylinder obstacle sits at the crossing point: every drone must route
    # around it with A* while also resolving conflicts with the other two.
    "obstacle_choke": {
        "astar_frame_size": 15.0,
        "obstacles": [dict(x=0.0, y=0.0, radius=6.0, z_min=0.0, z_max=30.0)],
        "vehicles": [
            dict(v_id="Q1", start=[-30, 0, ALT], end=[30, 0, ALT], speed=SPEED, mass_priority=2.0, margin=MARGIN),
            dict(v_id="Q2", start=[30, 3, ALT], end=[-30, -3, ALT], speed=SPEED, mass_priority=1.0, margin=MARGIN),
            dict(v_id="Q3", start=[0, -30, ALT], end=[0, 30, ALT], speed=SPEED, mass_priority=1.5, margin=MARGIN),
        ],
    },
}


def build_random(mod, sim_cls, n, seed=0, area_per_vehicle=400.0, astar_frame_size=15.0):
    """n vehicles with random starts/goals at constant density (default 400 m^2 each),
    for scaling tests. Starts are spaced >= 6 m apart so nobody begins in conflict."""
    import numpy as np
    rng = np.random.default_rng(seed)
    side = math.sqrt(n * area_per_vehicle)

    def spaced(k):
        pts = []
        while len(pts) < k:
            p = rng.uniform(0, side, 2)
            if all(np.hypot(*(p - q)) >= 6.0 for q in pts):
                pts.append(p)
        return pts

    starts, ends = spaced(n), spaced(n)
    sim = sim_cls(astar_frame_size=astar_frame_size, allow_3d=False)
    for k in range(n):
        sim.add_vehicle(mod.Vehicle(f"V{k}", [*starts[k], ALT], [*ends[k], ALT], SPEED,
                                    float(rng.choice([1.0, 2.0, 5.0, 10.0])), MARGIN))
    return sim


def build(mod, sim_cls, name):
    """Instantiate ``sim_cls`` (PHASE Simulation or PhaseStepper) for a scenario."""
    cfg = SCENARIOS[name]
    sim = sim_cls(astar_frame_size=cfg["astar_frame_size"], allow_3d=False)
    for o in cfg["obstacles"]:
        sim.add_obstacle(mod.CylinderObstacle(**o))
    for v in cfg["vehicles"]:
        sim.add_vehicle(mod.Vehicle(**v))
    return sim
