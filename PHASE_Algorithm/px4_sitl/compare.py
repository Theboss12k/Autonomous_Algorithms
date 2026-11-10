"""Side-by-side table of several analyzed run directories (any variants).

    python compare.py runs/<v12 run> runs/<phase_d run 1> runs/<phase_d run 2> > comparison.md
"""
import json
import sys
from pathlib import Path

import scenarios


def worst(pairs):
    p = min(pairs.values(), key=lambda x: x["min_sep"])
    return p["min_sep"], sum(x["violation_ticks"] for x in pairs.values())


def row(label, block, kinematic=False):
    pairs = block["pairs"] if kinematic else block["pairs_measured"]
    sep, viol = worst(pairs)
    vs = block["vehicles"]
    reached = all(v["reached"] for v in vs.values())
    delays = [v["delay_s"] for v in vs.values() if v["delay_s"] is not None]
    clear = [v["obstacle_clear_min_m"] for v in vs.values() if "obstacle_clear_min_m" in v]
    defl = block["deflections"]
    return (f"| {label} | {'yes' if reached else '**no**'} | {sep:.2f} | {viol} | "
            f"{max(delays):.1f} | {sum(delays) / len(delays):.1f} | "
            f"{(f'{min(clear):.2f}' if clear else '–')} | {defl} |")


def main(dirs):
    out = ["| run | all reached | worst separation (m, need 5.0) | ticks below margin | max delay s | mean delay s | min obstacle clearance (need 2.7) | deflections |",
           "|---|---|---|---|---|---|---|---|"]
    for name in scenarios.SCENARIOS:
        out.append(f"| **{name}** | | | | | | | |")
        found = []
        for d in dirs:
            rep_path = Path(d) / name / "report.json"
            if rep_path.exists():
                meta = json.loads((Path(d) / name / "meta.json").read_text())
                found.append((meta.get("variant", "v12"), Path(d), json.loads(rep_path.read_text())))
        # Kinematic baseline: from each variant's newest run directory (current code).
        latest = {}
        for variant, d, rep in sorted(found, key=lambda x: x[1].name):
            latest[variant] = (d, rep)
        for variant, d, rep in found:
            if latest.get(variant, (None,))[0] == d and not any(r.startswith(f"| {variant} kinematic") for r in out[-50:]
                                                              if r):
                pass
        shown = set()
        for variant, d, rep in found:
            if variant not in shown:
                ld, lrep = latest[variant]
                out.append(row(f"{variant} kinematic (code of `{ld.name}`)", lrep["kinematic"], kinematic=True))
                shown.add(variant)
            out.append(row(f"{variant} PX4 `{d.name}`", rep["px4"]))
    print("\n".join(out))


if __name__ == "__main__":
    main(sys.argv[1:])
