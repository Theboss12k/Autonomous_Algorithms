"""A crude stand-in for PX4's multicopter position control, for fast offline checks.

Each vehicle tracks PHASE's (position, velocity-feedforward) setpoint like PX4's
cascaded controller: vel_sp = ff + MPC_XY_P * pos_err, acc = MPC_XY_VEL_P_ACC *
vel_err, clamped to MPC_ACC_HOR_MAX, integrated at 50 Hz. It is not a flight
model; it only reproduces the tracking lag that broke PHASE-D's first PX4 runs.
"""
import numpy as np

XY_P = 0.95          # MPC_XY_P
VEL_P_ACC = 1.8      # MPC_XY_VEL_P_ACC
ACC_MAX = 5.0        # MPC_ACC_HOR_MAX
SUBSTEPS = 10


def fly(sim, max_t=150.0, variant="phase_d"):
    """Run ``sim`` (a PhaseStepper) closed loop against the lag plant."""
    sim.build_original_paths()
    pos = {v.v_id: np.array(v.start, dtype=float) for v in sim.vehicles}
    vel = {v.v_id: np.zeros(3) for v in sim.vehicles}
    trace = {v.v_id: [] for v in sim.vehicles}
    h = sim.dt / SUBSTEPS
    while sim.t < max_t and not sim.all_reached():
        targets = sim.step({k: p.copy() for k, p in pos.items()},
                           measured_vel={k: v.copy() for k, v in vel.items()})
        for v in sim.vehicles:
            vid = v.v_id
            sp = targets[vid]
            ff = np.copy(v.cmd_vel) if variant == "phase_d" else (sp - pos[vid]) / sim.dt
            if v.reached_goal:
                ff = np.zeros(3)
            for _ in range(SUBSTEPS):
                vel_sp = ff + XY_P * (sp - pos[vid])
                acc = VEL_P_ACC * (vel_sp - vel[vid])
                n = np.linalg.norm(acc[:2])
                if n > ACC_MAX:
                    acc[:2] *= ACC_MAX / n
                acc[2] = 0.0
                vel[vid] = vel[vid] + acc * h
                pos[vid] = pos[vid] + vel[vid] * h
            trace[vid].append((round(sim.t, 2), pos[vid].copy(), v.reached_goal))
    return trace


def min_separation(sim, trace):
    ids = [v.v_id for v in sim.vehicles]
    best = np.inf
    for k in range(len(trace[ids[0]])):
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                ta, tb = trace[ids[a]][k], trace[ids[b]][k]
                if ta[2] or tb[2]:
                    continue
                best = min(best, float(np.linalg.norm(ta[1][:2] - tb[1][:2])))
    return best
