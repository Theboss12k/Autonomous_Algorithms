# PHASE versions

Each folder holds one release of the algorithm, unchanged from when it was tested.

| File | Purpose |
|---|---|
| `phase_altitude_locked.py` | The algorithm in 2D (fixed-altitude) form. The test suite, PHASE-D and the PX4 scripts load this file. |
| `PHASE_Vx.x_Simulation.py` | The original stand-alone simulation, with its own scenario and Plotly view (V1.0 and V1.1). |
| `results/` | Test-suite results for that exact file (see below). |

## What changed between versions

- **V1.0** (Sep 2026): local A*. On conflict, either vehicle may sidestep 90° and follows that target before
  replanning. Micro-PBD (50 iterations) every tick. Same algorithm as the cited release
  (DOI 10.5281/zenodo.22260336); `PHASE_V1.0_Simulation.py` is byte-identical to the original
  `PHASE_Algorithm/PHASE_V1.0.py`.
- **V1.1**: each conflicting pair steers once per conflict event, and every vehicle replans with A* from its
  PBD-corrected position. PBD (1,000 iterations) runs **only when a new conflict appears**, which caused
  separation violations between events.
- **V1.2**: PBD every tick again; A* replans only on a new conflict; 1.5× momentum tie-break for equal-momentum
  pairs; reports vehicles stuck for more than 3 s.

## Results: provenance

`results/<case>/result.json` and `safety_report.txt` are copied from each version's final suite run. Plots and
per-tick path CSVs (about 1.4 GB in all) are left out; regenerate them with `test_suite/run_suite.py`.
`results/source_info.json` records the sha256 of the file that was tested, and it matches the
`phase_altitude_locked.py` beside it for all three versions.

| Version | Cases | Passed | Separation-violation ticks | Obstacle-penetration ticks | Notes |
|---|---|---|---|---|---|
| V1.0 | 53 (23 deterministic + 30 random) | 45 | 0 | 7 | Final run of 14 Sep. 120 older random cases from 13 Sep were dropped: they predate the final V1.0 file, and which revision produced them is not recorded. |
| V1.1 | 52 + summary of 53 | 29 of 52 | 118 | 4,266 | `run_summary.csv` comes from the full run (01:57). The 22 deterministic folders are a later re-run of the same file; the `23_original_manifest` folder was not kept. |
| V1.2 | 22 (deterministic only) | 18 | 0 | 0 | No random cases were run. `run_summary.csv` was rebuilt from the per-case files with `test_suite/summarize.py`, which reproduces V1.0's and V1.1's original summaries row for row. |

An earlier V1.2 run (`phase_test_results_v1.2.1`, from a different revision of the file) is not included.
