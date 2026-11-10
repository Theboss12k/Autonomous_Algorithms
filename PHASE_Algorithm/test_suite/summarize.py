#!/usr/bin/env python3
"""Rebuild run_summary.csv from the per-case result.json files of a suite run.

    python test_suite/summarize.py versions/V1.2/results
"""
import csv
import json
import sys
from pathlib import Path

COLUMNS = [
    "case_name", "category", "seed", "current_safety_checks_pass", "strict_diagnostic_pass",
    "all_goals_reached", "vehicles_reached", "vehicle_count", "center_to_center_violations",
    "obstacle_penetration_violations", "steering_events", "min_center_to_center_distance_m",
    "min_obstacle_clearance_m", "average_delay_s", "simulation_error",
]


def summarize(results_dir: Path) -> Path:
    rows = []
    for f in sorted(results_dir.glob("*/result.json")):
        s = json.loads(f.read_text())["summary"]
        rows.append({c: "" if s.get(c) is None else s.get(c) for c in COLUMNS})
    out = results_dir / "run_summary.csv"
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return out


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    path = summarize(Path(sys.argv[1]))
    print(f"wrote {path}")
