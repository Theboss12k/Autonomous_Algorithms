"""With perfect tracking, PhaseStepper.step() must reproduce Simulation.run()."""
import contextlib
import io
import os

import numpy as np
import pytest

from phase_d import scenarios
from phase_d.phase_stepper import load_phase, make_stepper_class

MOD = load_phase()
Stepper = make_stepper_class(MOD)
MAX_T = 300.0  # Simulation.run() always simulates 300 s


def _reference(name, tmp_path):
    sim = scenarios.build(MOD, MOD.Simulation, name)
    cwd = os.getcwd()
    os.chdir(tmp_path)  # run() writes resolved_paths.csv into the cwd
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            sim.run()
    finally:
        os.chdir(cwd)
    return sim


@pytest.mark.parametrize("name", list(scenarios.SCENARIOS))
@pytest.mark.parametrize("feed_measured", [False, True])
def test_step_matches_run(name, feed_measured, tmp_path):
    ref = _reference(name, tmp_path)

    st = scenarios.build(MOD, Stepper, name)
    st.build_original_paths()
    while st.t < MAX_T:  # same float-accumulating condition as run()
        # feed_measured: pass back exactly what PHASE committed last tick, i.e. a
        # perfectly tracking vehicle. Must be a no-op versus the internal loop.
        measured = {v.v_id: np.copy(v.current_pos) for v in st.vehicles} if feed_measured else None
        st.step(measured)

    for rv, sv in zip(ref.vehicles, st.vehicles):
        assert len(rv.resolved_path) == len(sv.resolved_path), rv.v_id
        r = np.array([p['pos'] for p in rv.resolved_path])
        s = np.array([p['pos'] for p in sv.resolved_path])
        np.testing.assert_allclose(s, r, atol=1e-9, err_msg=rv.v_id)
        assert rv.state_history == sv.state_history
        assert rv.new_time == pytest.approx(sv.new_time)
    assert len(ref.deflection_events) == len(st.deflection_events)
