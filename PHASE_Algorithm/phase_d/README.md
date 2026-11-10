# PHASE-D (experimental)

Dynamics-aware PHASE. It builds on V1.2 and adds:
- growing safety balls (reaction plus braking distance);
- priority braking, where the lower-momentum vehicle yields first;
- acceleration-limited commands;
- deflections that run to completion;
- neighbour search on a spatial grid.

**Design, equations and results: [`PHASE_D.md`](PHASE_D.md).**

| File | Role |
|---|---|
| `phase_dynamic.py` | PHASE-D. The per-tick logic is `make_dynamic_stepper_class()` → `_tick()` |
| `spatial_grid.py` | Uniform spatial hash grid: O(N·k) neighbour and pair queries instead of O(N²) |
| `phase_stepper.py` | Loads any PHASE version from `../versions` unchanged and steps it one tick at a time (also used by the PX4 bridge) |
| `lag_plant.py` | Crude PX4-like tracking model for fast offline closed-loop checks |
| `scenarios.py` | Quad-scale scenarios: `three_way_swap`, `head_on_cross`, `obstacle_choke` |
| `bench_scaling.py` | Tick cost vs fleet size, grid vs every pair (`python -m phase_d.bench_scaling`) |
| `tests/` | 48 tests: stepper ≡ `Simulation.run()`, every PHASE-D rule, grid ≡ brute force, lag-model safety |

```python
from phase_d.phase_stepper import load_phase
from phase_d.phase_dynamic import make_dynamic_stepper_class
from phase_d import scenarios

mod = load_phase()                       # versions/V1.2 by default (PHASE_SRC overrides)
Sim = make_dynamic_stepper_class(mod)
sim = scenarios.build(mod, Sim, "head_on_cross")
while not sim.all_reached():
    sim.step()                           # kinematic; pass measured=… for closed loop
```

Run from `PHASE_Algorithm/` so that `phase_d` is importable.
