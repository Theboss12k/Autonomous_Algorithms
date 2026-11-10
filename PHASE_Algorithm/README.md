# PHASE: Physics-based Hierarchical Adaptive Spatial Evasion

PHASE coordinates many autonomous vehicles sharing the same space. Each vehicle plans its own route with
local A*. When two vehicles' safety bubbles overlap, the lower-momentum one sidesteps (like a lighter
billiard ball), and a geometric solver (PBD) pushes any remaining overlap apart.

- **New here?** Open [`ROADMAP.html`](ROADMAP.html) in a browser for the story of every version, with results.
- **The algorithm in depth:** [`Algorithm_Overview.md`](Algorithm_Overview.md)
- **What changed and why:** [`CHANGELOG.md`](CHANGELOG.md)

## Where things are

```text
PHASE_Algorithm/
├── versions/                 the algorithm, one folder per release
│   ├── V1.0/                 phase_altitude_locked.py  ·  PHASE_V1.0_Simulation.py  ·  results/
│   ├── V1.1/                 phase_altitude_locked.py  ·  PHASE_V1.1_Simulation.py  ·  results/
│   └── V1.2/                 phase_altitude_locked.py  ·  results/          ← newest
├── test_suite/               kinematic test suite (runs any version)
├── phase_d/                  PHASE-D: dynamics-aware variant + its tests  (design: phase_d/PHASE_D.md)
├── px4_sitl/                 fly PHASE or PHASE-D on 3 PX4 SIH quadcopters
├── ROADMAP.html              visual history of the versions
├── CHANGELOG.md              every PX4 / PHASE-D change, with evidence
└── requirements.txt
```

| Version | Status | What it is | Kinematic suite (22 shared cases) |
|---|---|---|---|
| [V1.0](versions/V1.0) | stable, cited ([DOI](https://doi.org/10.5281/zenodo.22260336)) | Local A*, 90° evasion, PBD every tick | 18 passed, 0 separation violations, mean delay 7.3 s |
| [V1.1](versions/V1.1) | superseded | Steer once per conflict, global replan; PBD only on new conflicts | 14 passed, **54 violation ticks** |
| [V1.2](versions/V1.2) | **newest** | PBD every tick again, replan on new conflicts, deadlock detection | 18 passed, 0 violations, mean delay 3.7 s |
| [PHASE-D](phase_d) | experimental | V1.2 + growing balls, priority braking, persistent deflections, spatial grid | 0 violations in 6 PX4 SIH flights (V1.2: 11–24 per flight) |

`phase_altitude_locked.py` in each version folder is the 2D (fixed altitude) form used by the test suite and
by PX4. `PHASE_Vx.x_Simulation.py` is the original stand-alone simulation with its own scenario and plot.

## Quick start

```bash
cd PHASE_Algorithm
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

| I want to… | Command |
|---|---|
| Run a version's own simulation (opens a plot) | `.venv/bin/python versions/V1.0/PHASE_V1.0_Simulation.py` |
| Run the test suite on a version | `.venv/bin/python test_suite/run_suite.py V1.2 --random 0` |
| Run the PHASE-D tests (48) | `.venv/bin/python -m pytest` |
| Benchmark PHASE-D, grid vs every pair | `.venv/bin/python -m phase_d.bench_scaling` |
| Fly on PX4 quadcopters | see [`px4_sitl/README.md`](px4_sitl/README.md) (needs a PX4 build) |

PHASE-D and the PX4 scripts run V1.2 by default. Set `PHASE_SRC` to another version name (`PHASE_SRC=V1.1`)
or to the path of any PHASE file to change that.

## Which results are which

- **Kinematic** (`versions/*/results/`, `test_suite/`): PHASE's own simulator. Vehicles move exactly where
  the algorithm puts them, so these runs test the logic, not flight.
- **PX4 SIH** (`px4_sitl/`): the real PX4 flight stack with its built-in physics model, run on one computer.
  It is software-in-the-loop only: no Gazebo, no hardware.

## Citation

See [`../CITATION.cff`](../CITATION.cff) (DOI [10.5281/zenodo.22260336](https://doi.org/10.5281/zenodo.22260336)).
