"""PHASE-D tick cost vs fleet size: spatial grid vs O(N^2) brute force.

Random fleets at constant density (400 m^2 per vehicle), kinematic mode.
Tick 1 (initial A* for everyone) is reported separately; the rest are averaged.

    python -m phase_d.bench_scaling [--sizes 10 30 100 300 1000] [--brute-max 300] [--ticks 8]
"""
import argparse
import json
import time

from . import scenarios
from .phase_dynamic import make_dynamic_stepper_class
from .phase_stepper import load_phase


def bench(mod, n, use_grid, ticks):
    cls = make_dynamic_stepper_class(mod, use_grid=use_grid)
    sim = scenarios.build_random(mod, cls, n, seed=1)
    sim.build_original_paths()
    t0 = time.perf_counter()
    sim.step()
    first = time.perf_counter() - t0
    before = sim.counters["pair_checks"]
    t0 = time.perf_counter()
    for _ in range(ticks):
        sim.step()
    rest = (time.perf_counter() - t0) / ticks
    pairs = (sim.counters["pair_checks"] - before) / ticks
    astar = sum(tk["astar_calls"] for tk in sim.tick_log[1:]) / ticks
    return {"n": n, "grid": use_grid, "first_tick_s": round(first, 3), "tick_s": round(rest, 4),
            "pairs_per_tick": round(pairs), "astar_per_tick": round(astar, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=int, nargs="+", default=[10, 30, 100, 300, 1000])
    ap.add_argument("--brute-max", type=int, default=300)
    ap.add_argument("--ticks", type=int, default=8)
    args = ap.parse_args()
    mod = load_phase()
    rows = []
    for n in args.sizes:
        for g in (True, False):
            if not g and n > args.brute_max:
                continue
            r = bench(mod, n, g, args.ticks)
            rows.append(r)
            print(json.dumps(r), flush=True)
    print("\n| vehicles | method | pair checks / tick | ms / tick (after tick 1) | first tick s (initial A*) |")
    print("|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['n']} | {'grid' if r['grid'] else 'brute force'} | {r['pairs_per_tick']:,} | "
              f"{1e3 * r['tick_s']:.1f} | {r['first_tick_s']} |")


if __name__ == "__main__":
    main()
