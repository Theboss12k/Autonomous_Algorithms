# PHASE test suite (kinematic)

A test harness that loads a PHASE file **unchanged** and runs adversarial and random scenarios on it:
- 23 deterministic cases: head-on, three-way and four-way crossings, momentum and speed extremes, obstacle
  fields, a tight corridor, swapping, and the original 41-vehicle manifest;
- seeded random families: mixed traffic, crossings and convergence.

All three versions were tested with this same file. It was identical in each version's folder, so it is kept here once.

```bash
python test_suite/run_suite.py V1.2                   # deterministic + 10 random cases per family
python test_suite/run_suite.py V1.2 --random 0        # deterministic only
python test_suite/run_suite.py V1.0 --cases 02_head_on,04_three_way_cross --skip-original
python test_suite/summarize.py test_suite/output/V1.2 # rebuild run_summary.csv from result.json files
```

Output goes to `test_suite/output/<version>/` (git-ignored). Each case gets:
- `result.json`, with pass/fail, separation and obstacle violations, delays and per-vehicle metrics;
- `safety_report.txt`;
- path CSVs and a plot.

**Pass criteria:**
- `current_safety_checks_pass`: every vehicle reached its goal and no vehicle centres came closer than their margins.
- `strict_diagnostic_pass` additionally requires no obstacle penetration.

Saved results for each version are in `../versions/<version>/results/`.
