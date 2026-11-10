"""Metrics and plots for PHASE runs: PX4 closed loop vs PHASE's own kinematic sim.

    python analyze.py runs/<dir>                # every scenario sub-folder with a flight.csv
    python analyze.py runs/<dir> --baseline-only

For each scenario it also runs the stock ``Simulation.run()`` (the kinematic
baseline, no PX4) and writes ``<scenario>/report.json``, ``<scenario>/paths.html``
and a combined ``summary.md`` in the run directory.
"""
import argparse
import collections
import contextlib
import csv
import io
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # PHASE_Algorithm/, home of phase_d
from phase_d import scenarios
from phase_d.phase_dynamic import make_dynamic_stepper_class
from phase_d.phase_stepper import load_phase

VIOLATION_TOL = 0.05  # same tolerance as PHASE's verify_manifest()


def pairwise(series, margins, active):
    """series: {v_id: [(t, xyz)]} on a shared timeline -> per-pair min sep & violations."""
    ids = sorted(series)
    times = sorted({t for s in series.values() for t, _ in s})
    pos = {vid: dict(s) for vid, s in series.items()}
    out = {}
    for a_i in range(len(ids)):
        for b_i in range(a_i + 1, len(ids)):
            a, b = ids[a_i], ids[b_i]
            need = margins[a] + margins[b]
            best, best_t, viol = np.inf, None, 0
            for t in times:
                if t not in pos[a] or t not in pos[b] or not (active(a, t) and active(b, t)):
                    continue
                d = float(np.linalg.norm(pos[a][t][:2] - pos[b][t][:2]))
                if d < best:
                    best, best_t = d, t
                if d < need - VIOLATION_TOL:
                    viol += 1
            out[f"{a}-{b}"] = {"min_sep": round(best, 3), "at_t": best_t, "required": need,
                               "violation_ticks": viol}
    return out


def path_len(pts):
    pts = np.asarray(pts)
    return float(np.sum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))) if len(pts) > 1 else 0.0


def baseline(mod, name, variant="v12"):
    """Kinematic run (no PX4) of the same PHASE variant the flight used."""
    if variant == "phase_d":
        sim = scenarios.build(mod, make_dynamic_stepper_class(mod), name)
        sim.build_original_paths()
        while sim.t < 300.0 and not sim.all_reached():
            sim.step()
        counters = dict(sim.counters)
    else:
        sim = scenarios.build(mod, mod.Simulation, name)
        counters = {}
        cwd = os.getcwd()
        tmp = Path(os.environ.get("TMPDIR", "/tmp")) / f"phase_baseline_{os.getpid()}"
        tmp.mkdir(parents=True, exist_ok=True)
        os.chdir(tmp)  # run() writes resolved_paths.csv into the cwd
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                sim.run()
        finally:
            os.chdir(cwd)
    margins = {v.v_id: v.margin for v in sim.vehicles}
    done = {v.v_id: (v.new_time or np.inf) for v in sim.vehicles}
    series = {v.v_id: [(round(p['t'], 2), np.asarray(p['pos'])) for p in v.resolved_path] for v in sim.vehicles}
    pairs = pairwise(series, margins, lambda vid, t: 0 < t <= done[vid] + 1e-9)
    per_v = {}
    for v in sim.vehicles:
        pts = [p for t, p in series[v.v_id] if t <= done[v.v_id] + 1e-9]
        per_v[v.v_id] = {
            "reached": v.new_time > 0, "ideal_s": round(v.orig_time, 2),
            "actual_s": round(v.new_time, 2) if v.new_time else None,
            "delay_s": round(max(v.new_time - v.orig_time, 0), 2) if v.new_time else None,
            "path_m": round(path_len(pts), 2), "straight_m": round(v.orig_dist, 2),
        }
        if sim.obstacles:
            per_v[v.v_id]["obstacle_clear_min_m"] = round(min(
                float(np.linalg.norm(p[:2] - np.array([o.x, o.y]))) - o.radius for p in pts for o in sim.obstacles), 2)
            per_v[v.v_id]["obstacle_clear_required_m"] = v.margin + 0.2
    return {"pairs": pairs, "vehicles": per_v, "deflections": len(sim.deflection_events),
            "deadlock_boosts": sorted(map(list, sim.identified_deadlocks)), "counters": counters,
            "series": series}


def flight(run_dir):
    meta = json.loads((run_dir / "meta.json").read_text())
    ticks = json.loads((run_dir / "phase_ticks.json").read_text())
    rows = list(csv.DictReader(open(run_dir / "flight.csv")))
    margins = {v["v_id"]: v["margin"] for v in meta["config"]["vehicles"]}
    meas, cmd, reached_at = collections.defaultdict(list), collections.defaultdict(list), {}
    track_err = collections.defaultdict(list)
    for r in rows:
        t, vid = float(r["t"]), r["v_id"]
        m = np.array([float(r["meas_x"]), float(r["meas_y"]), float(r["meas_z"])])
        c = np.array([float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])])
        meas[vid].append((t, m))
        cmd[vid].append((t, c))
        if r["reached"] == "1":
            reached_at.setdefault(vid, t)
    # Tracking error: the target PHASE commanded at tick k is where it wants the
    # drone at tick k+1, so compare it with the measurement one tick later.
    for vid in meas:
        for (t, c), (_, m_next) in zip(cmd[vid], meas[vid][1:]):
            if t < reached_at.get(vid, np.inf):
                track_err[vid].append(float(np.linalg.norm(c[:2] - m_next[:2])))
    done = {vid: reached_at.get(vid, np.inf) for vid in meas}
    active = lambda vid, t: t < done[vid]  # noqa: E731
    per_v = {}
    for v in meta["config"]["vehicles"]:
        vid = v["v_id"]
        ideal = meta["orig_time"][vid]
        pts = [p for t, p in meas[vid] if t <= done[vid]]
        alt = [p[2] for t, p in meas[vid] if t <= done[vid]]
        per_v[vid] = {
            "reached": vid in reached_at, "ideal_s": round(ideal, 2),
            "actual_s": reached_at.get(vid), "delay_s": round(max(reached_at[vid] - ideal, 0), 2) if vid in reached_at else None,
            "path_m": round(path_len(pts), 2),
            "track_err_mean_m": round(float(np.mean(track_err[vid])), 3) if track_err[vid] else None,
            "track_err_max_m": round(float(np.max(track_err[vid])), 3) if track_err[vid] else None,
            "alt_dev_max_m": round(float(np.max(np.abs(np.array(alt) - v["start"][2]))), 3) if alt else None,
        }
        obs = meta["config"]["obstacles"]
        if obs:
            # Clearance beyond the obstacle surface; PHASE requires >= margin + 0.2.
            per_v[vid]["obstacle_clear_min_m"] = round(min(
                float(np.linalg.norm(p[:2] - np.array([o["x"], o["y"]]))) - o["radius"] for p in pts for o in obs), 2)
            per_v[vid]["obstacle_clear_required_m"] = v["margin"] + 0.2
    compute = [tk["compute_s"] for tk in ticks]
    return {
        "pairs_measured": pairwise(meas, margins, active),
        "pairs_commanded": pairwise(cmd, margins, active),
        "vehicles": per_v,
        "deflections": len(meta["deflection_events"]),
        "deadlock_boosts": meta["identified_deadlocks"],
        "replans": int(sum(tk["astar_calls"] for tk in ticks)),
        "pbd_ticks": int(sum(1 for tk in ticks if any(s > 1e-6 for s in tk["pbd_shift"].values()))),
        "pbd_max_shift_m": round(max((max(tk["pbd_shift"].values(), default=0) for tk in ticks), default=0), 3),
        "phase_compute_ms": {"mean": round(1e3 * float(np.mean(compute)), 2), "max": round(1e3 * float(np.max(compute)), 2)},
        "loop_overruns": meta["overruns"], "vel_clamps": meta["vel_clamps"],
        "left_offboard": any(r["flight_mode"] != "OFFBOARD" for r in rows),
        "series_meas": meas, "series_cmd": cmd, "meta": meta,
    }


def plot(name, cfg, base, fl, path):
    import plotly.graph_objects as go
    fig = go.Figure()
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for o in cfg["obstacles"]:
        th = np.linspace(0, 2 * np.pi, 60)
        fig.add_trace(go.Scatter(x=o["x"] + o["radius"] * np.cos(th), y=o["y"] + o["radius"] * np.sin(th),
                                 fill="toself", mode="lines", line=dict(color="#888"), name="obstacle"))
    for k, v in enumerate(cfg["vehicles"]):
        vid, c = v["v_id"], colors[k % 3]
        fig.add_trace(go.Scatter(x=[v["start"][0], v["end"][0]], y=[v["start"][1], v["end"][1]], mode="lines",
                                 line=dict(color=c, dash="dot", width=1), name=f"{vid} straight", legendgroup=vid))
        b = np.array([p for _, p in base["series"][vid]])
        fig.add_trace(go.Scatter(x=b[:, 0], y=b[:, 1], mode="lines", line=dict(color=c, dash="dash", width=2),
                                 opacity=0.5, name=f"{vid} PHASE kinematic", legendgroup=vid))
        if fl:
            m = np.array([p for _, p in fl["series_meas"][vid]])
            fig.add_trace(go.Scatter(x=m[:, 0], y=m[:, 1], mode="lines", line=dict(color=c, width=3),
                                     name=f"{vid} PX4 flown", legendgroup=vid))
        fig.add_trace(go.Scatter(x=[v["start"][0]], y=[v["start"][1]], mode="markers+text", text=[vid],
                                 textposition="top center", marker=dict(color=c, size=9), showlegend=False))
        fig.add_trace(go.Scatter(x=[v["end"][0]], y=[v["end"][1]], mode="markers", marker=dict(color=c, size=10, symbol="x"),
                                 showlegend=False))
    fig.update_layout(title=f"{name}: PX4 SIH closed loop vs PHASE kinematic", xaxis_title="East (m)",
                      yaxis_title="North (m)", yaxis=dict(scaleanchor="x"), template="plotly_white")
    fig.write_html(path, include_plotlyjs="cdn")


def strip(d):
    return {k: v for k, v in d.items() if not k.startswith("series") and k != "meta"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--baseline-only", action="store_true")
    ap.add_argument("--variant", choices=["v12", "phase_d"], default="v12", help="for --baseline-only")
    args = ap.parse_args()
    root = Path(args.run_dir)
    mod = load_phase()
    lines = [f"# PHASE x PX4 SIH results\n", f"PHASE source: `{mod.__phase_path__}` (sha256 `{mod.__phase_sha256__[:12]}`)\n"]
    for name, cfg in scenarios.SCENARIOS.items():
        sd = root / name
        fl = flight(sd) if (sd / "flight.csv").exists() and not args.baseline_only else None
        if fl is None and not args.baseline_only:
            continue
        sd.mkdir(parents=True, exist_ok=True)
        variant = fl["meta"].get("variant", "v12") if fl else args.variant
        base = baseline(mod, name, variant)
        report = {"scenario": name, "kinematic": strip(base), "px4": strip(fl) if fl else None}
        (sd / "report.json").write_text(json.dumps(report, indent=2, default=float))
        plot(name, cfg, base, fl, sd / "paths.html")

        lines.append(f"\n## {name} ({variant})\n")
        lines.append("| pair | required | kinematic min | PX4 commanded min | PX4 measured min | measured violation ticks |")
        lines.append("|---|---|---|---|---|---|")
        for pair, kb in base["pairs"].items():
            pc = fl["pairs_commanded"][pair] if fl else {}
            pm = fl["pairs_measured"][pair] if fl else {}
            lines.append(f"| {pair} | {kb['required']:.1f} | {kb['min_sep']:.2f} | {pc.get('min_sep', '-')} | "
                         f"{pm.get('min_sep', '-')} | {pm.get('violation_ticks', '-')} |")
        lines.append("\n| vehicle | ideal s | kinematic s | PX4 s | PX4 delay s | path m (kin / PX4) | track err mean / max m |")
        lines.append("|---|---|---|---|---|---|---|")
        for vid, kv in base["vehicles"].items():
            pv = fl["vehicles"][vid] if fl else {}
            lines.append(f"| {vid} | {kv['ideal_s']} | {kv['actual_s']} | {pv.get('actual_s', '-')} | {pv.get('delay_s', '-')} | "
                         f"{kv['path_m']} / {pv.get('path_m', '-')} | {pv.get('track_err_mean_m', '-')} / {pv.get('track_err_max_m', '-')} |")
        if fl:
            lines.append(f"\nDeflections kinematic {base['deflections']} / PX4 {fl['deflections']}; "
                         f"A* calls {fl['replans']}; PBD active ticks {fl['pbd_ticks']} (max shift {fl['pbd_max_shift_m']} m); "
                         f"PHASE compute {fl['phase_compute_ms']['mean']} ms mean / {fl['phase_compute_ms']['max']} ms max; "
                         f"loop overruns {fl['loop_overruns']}; velocity clamps {fl['vel_clamps']}; "
                         f"left offboard: {fl['left_offboard']}; sim/wall clock {fl['meta'].get('sim_over_wall', '?')}.")
            if fl["meta"].get("counters"):
                lines.append(f"\nPHASE-D counters, kinematic: {base['counters']}; PX4: {fl['meta']['counters']}")
            oc = {vid: pv.get("obstacle_clear_min_m") for vid, pv in fl["vehicles"].items() if "obstacle_clear_min_m" in pv}
            if oc:
                req = next(iter(fl["vehicles"].values()))["obstacle_clear_required_m"]
                lines.append(f"\nPX4 measured obstacle clearance beyond surface (required {req:.1f} m): "
                             + ", ".join(f"{k} {v} m" for k, v in oc.items()))
    (root / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
