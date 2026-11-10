#!/usr/bin/env python3
"""
PHASE Autonomous Test Suite

Loads the supplied PHASE implementation as-is and runs the deterministic
and randomized scenarios. The harness does not modify PHASE classes/functions.

Outputs are stored one folder per scenario. Plot files are named after the
scenario itself.

Default:
    python phase_autonomous_test_suite.py

Examples:
    python phase_autonomous_test_suite.py --random 0
    python phase_autonomous_test_suite.py --random 50
    python phase_autonomous_test_suite.py --cases head_on,three_way_cross
    python phase_autonomous_test_suite.py --phase-file /path/to/phase_altitude_locked.py
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
import random
import shutil
import statistics
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional

import numpy as np


# ============================================================
# Load PHASE source without modifying it
# ============================================================

def load_module_from_file(path: Path):
    path = path.resolve()
    module_name = "phase_under_test"

    spec = importlib.util.spec_from_file_location(module_name, str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load PHASE module from: {path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# ============================================================
# Test-case definition
# ============================================================

@dataclass
class TestCase:
    name: str
    description: str
    builder: Callable
    category: str = "deterministic"
    seed: Optional[int] = None


# ============================================================
# Scenario builders
# ============================================================

def blank_sim(P):
    return P.Simulation(astar_frame_size=8.0, allow_3d=False)


def case_no_collision(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 0, 10], [0, 100, 10], 10, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 0, 10], [100, 100, 10], 10, 10, 4, 0))
    return sim


def case_head_on(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 50, 10], [100, 50, 10], 20, 100, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 50, 10], [0, 50, 10], 10, 1, 5, 0))
    return sim


def case_equal_head_on(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 50, 10], [100, 50, 10], 15, 10, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 50, 10], [0, 50, 10], 15, 10, 5, 0))
    return sim


def case_three_way_cross(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 50, 10], [100, 50, 10], 12, 10, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [50, 0, 10], [50, 100, 10], 12, 10, 5, 0))
    sim.add_vehicle(P.Vehicle("C", [100, 100, 10], [0, 0, 10], 12, 10, 5, 0))
    return sim


def case_four_way_cross(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("NORTH", [50, 0, 10], [50, 100, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("SOUTH", [50, 100, 10], [50, 0, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("EAST", [0, 50, 10], [100, 50, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("WEST", [100, 50, 10], [0, 50, 10], 18, 10, 4, 0))
    return sim


def case_diagonal_cross(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 0, 10], [100, 100, 10], 20, 5, 4, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 0, 10], [0, 100, 10], 20, 5, 4, 0))
    return sim


def case_momentum_extreme(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("HEAVY", [0, 50, 10], [100, 50, 10], 30, 1_000_000, 5, 0))
    sim.add_vehicle(P.Vehicle("LIGHT", [100, 50, 10], [0, 50, 10], 30, 1e-6, 5, 0))
    return sim


def case_speed_extreme(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("FAST", [0, 0, 10], [100, 0, 10], 250, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("FAST_CROSS", [50, -50, 10], [50, 85, 10], 250, 1, 5, 0))
    return sim


def case_staggered_injection(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 48, 10], [100, 48, 10], 28, 1, 4, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 52, 10], [0, 52, 10], 28, 1, 4, 8.1))
    sim.add_vehicle(P.Vehicle("C", [50, 0, 10], [50, 100, 10], 25, 0.01, 5, 12))
    return sim


def case_chain(P):
    sim = blank_sim(P)
    chain_positions = [
        [0, 150, 10], [24, 150, 10], [48, 150, 10],
        [72, 150, 10], [96, 150, 10]
    ]

    for i, pos in enumerate(chain_positions):
        sim.add_vehicle(P.Vehicle(
            f"CHAIN_{i}", pos, [pos[0], 250, 10],
            30, 1, 12, 0
        ))

    return sim


def case_pbd_cluster(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("PBD_A", [200, 0, 10], [300, 0, 10], 10, 1, 20, 0))
    sim.add_vehicle(P.Vehicle("PBD_B", [201, 0, 10], [301, 0, 10], 10, 1, 20, 0))
    sim.add_vehicle(P.Vehicle("PBD_C", [202, 0, 10], [302, 0, 10], 10, 1, 20, 0))
    return sim


def case_zero_distance(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [350, 50, 10], [450, 50, 10], 15, 1, 8, 0))
    sim.add_vehicle(P.Vehicle("B", [350, 50, 10], [350, 150, 10], 15, 1, 8, 0))
    return sim


def case_obstacle_field(P):
    sim = blank_sim(P)
    sim.add_obstacle(P.CylinderObstacle(50, 50, 25, 0, 20))
    sim.add_obstacle(P.CylinderObstacle(25, 50, 15, 0, 20))
    sim.add_obstacle(P.CylinderObstacle(75, 50, 15, 0, 20))
    sim.add_obstacle(P.CylinderObstacle(50, 25, 15, 0, 20))
    sim.add_obstacle(P.CylinderObstacle(50, 75, 15, 0, 20))
    sim.add_vehicle(P.Vehicle("A", [0, 50, 10], [100, 50, 10], 25, 100, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 50, 10], [0, 50, 10], 25, 0.001, 5, 0))
    return sim


def case_inside_obstacle(P):
    sim = blank_sim(P)
    sim.add_obstacle(P.CylinderObstacle(50, 50, 25, 0, 20))
    sim.add_vehicle(P.Vehicle("INSIDE", [10, 50, 10], [150, 50, 10], 15, 1, 5, 0))
    return sim


def case_bad_goal(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("BAD_GOAL", [0, 50, 10], [10, 50, 10], 15, 1, 5, 0))
    return sim


def case_grazing(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 82, 10], [100, 82, 10], 20, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [0, 83, 10], [100, 83, 10], 20, 1, 5, 0))
    return sim


def case_vertical_2d(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("CLIMB", [0, 20, 5], [100, 80, 40], 15, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("DESCEND", [100, 80, 40], [0, 20, 5], 15, 10, 4, 0))
    return sim


def case_low_high(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("LOW", [0, 60, 2], [100, 60, 2], 22, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("HIGH", [100, 60, 18], [0, 60, 18], 22, 10, 4, 0))
    return sim


def case_parallel_close(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 0, 10], [120, 0, 10], 15, 10, 10, 0))
    sim.add_vehicle(P.Vehicle("B", [0, 1, 10], [120, 1, 10], 15, 10, 10, 0))
    sim.add_vehicle(P.Vehicle("C", [0, -1, 10], [120, -1, 10], 15, 1, 10, 0))
    return sim


def case_tight_corridor(P):
    sim = blank_sim(P)
    sim.add_obstacle(P.CylinderObstacle(50, 15, 10, 0, 20))
    sim.add_obstacle(P.CylinderObstacle(50, -15, 10, 0, 20))
    sim.add_vehicle(P.Vehicle("A", [0, 0, 10], [100, 0, 10], 12, 100, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [100, 0, 10], [0, 0, 10], 10, 1, 5, 0))
    return sim


def case_late_cross(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [0, 50, 10], [100, 50, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("B", [50, 0, 10], [50, 100, 10], 18, 10, 4, 5))
    sim.add_vehicle(P.Vehicle("C", [100, 50, 10], [0, 50, 10], 18, 0.1, 4, 10))
    return sim


def case_swapping(P):
    sim = blank_sim(P)
    sim.add_vehicle(P.Vehicle("A", [-20, 0, 10], [20, 0, 10], 80, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("B", [20, 0, 10], [-20, 0, 10], 80, 1, 5, 0))
    return sim


def case_original_manifest(P):
    sim = P.Simulation(astar_frame_size=8.0, allow_3d=False)

    sim.add_obstacle(P.CylinderObstacle(x=50, y=50, radius=25, z_min=0, z_max=20))
    sim.add_obstacle(P.CylinderObstacle(x=25, y=50, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(P.CylinderObstacle(x=75, y=50, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(P.CylinderObstacle(x=50, y=25, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(P.CylinderObstacle(x=50, y=75, radius=15, z_min=0, z_max=20))

    sim.add_vehicle(P.Vehicle("HEAD_A", [0, 50, 10], [100, 50, 10], 25, 1000, 5, 0))
    sim.add_vehicle(P.Vehicle("HEAD_B", [100, 50, 10], [0, 50, 10], 25, 0.001, 5, 0))
    sim.add_vehicle(P.Vehicle("TANK", [0, 30, 10], [100, 30, 10], 30, 100000, 6, 0))
    sim.add_vehicle(P.Vehicle("FEATHER", [100, 30, 10], [0, 30, 10], 5, 0.0001, 3, 0))

    sim.add_vehicle(P.Vehicle("STACK_A", [10, 10, 10], [90, 90, 10], 12, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("STACK_B", [10, 10, 10], [90, 10, 10], 12, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("STACK_C", [10, 10, 10], [10, 90, 10], 12, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("STACK_D", [10, 10, 10], [96, 50, 10], 12, 1, 5, 0))

    sim.add_vehicle(P.Vehicle("NORTH", [50, 0, 10], [50, 100, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("SOUTH", [50, 100, 10], [50, 0, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("EAST", [0, 50, 10], [100, 50, 10], 18, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("WEST", [100, 50, 10], [0, 50, 10], 18, 10, 4, 0))

    sim.add_vehicle(P.Vehicle("DIAG_A", [0, 0, 10], [100, 100, 10], 20, 5, 4, 0))
    sim.add_vehicle(P.Vehicle("DIAG_B", [100, 0, 10], [0, 100, 10], 20, 5, 4, 0))

    sim.add_vehicle(P.Vehicle("CLIMB", [0, 20, 5], [100, 80, 40], 15, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("DESCEND", [100, 80, 40], [0, 20, 5], 15, 10, 4, 0))

    sim.add_vehicle(P.Vehicle("LOW_CRUISER", [0, 60, 2], [100, 60, 2], 22, 10, 4, 0))
    sim.add_vehicle(P.Vehicle("HIGH_CRUISER", [100, 60, 18], [0, 60, 18], 22, 10, 4, 0))

    sim.add_vehicle(P.Vehicle("INJECT_A", [0, 48, 10], [100, 48, 10], 28, 1, 4, 8))
    sim.add_vehicle(P.Vehicle("INJECT_B", [100, 52, 10], [0, 52, 10], 28, 1, 4, 8.1))
    sim.add_vehicle(P.Vehicle("LATE_CROSS", [50, 0, 10], [50, 100, 10], 25, 0.01, 5, 12))

    sim.add_vehicle(P.Vehicle("FAST_X", [0, 0, 10], [100, 0, 10], 250, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("FAST_Y", [50, -50, 10], [50, 85, 10], 250, 1, 5, 0))

    sim.add_vehicle(P.Vehicle("SWAP_A", [-20, 0, 10], [20, 0, 10], 200, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("SWAP_B", [20, 0, 10], [-20, 0, 10], 200, 1, 5, 0))

    chain_positions = [
        [0, 150, 10], [24, 150, 10], [48, 150, 10],
        [72, 150, 10], [96, 150, 10]
    ]
    for i, p in enumerate(chain_positions):
        sim.add_vehicle(P.Vehicle(
            f"CHAIN_{i}", p, [p[0], 250, 10],
            30, 1, 12, 0
        ))

    sim.add_vehicle(P.Vehicle("PBD_A", [200, 0, 10], [300, 0, 10], 10, 1, 20, 0))
    sim.add_vehicle(P.Vehicle("PBD_B", [201, 0, 10], [301, 0, 10], 10, 1, 20, 0))
    sim.add_vehicle(P.Vehicle("PBD_C", [202, 0, 10], [302, 0, 10], 10, 1, 20, 0))

    sim.add_vehicle(P.Vehicle("ZERO_A", [350, 50, 10], [450, 50, 10], 15, 1, 8, 0))
    sim.add_vehicle(P.Vehicle("ZERO_B", [350, 50, 10], [350, 150, 10], 15, 1, 8, 0))

    sim.add_vehicle(P.Vehicle("INSIDE_OBS", [10, 50, 10], [150, 50, 10], 15, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("BAD_GOAL", [0, 50, 10], [10, 50, 10], 15, 1, 5, 0))

    sim.add_vehicle(P.Vehicle("GRAZER_A", [0, 82, 10], [100, 82, 10], 20, 1, 5, 0))
    sim.add_vehicle(P.Vehicle("GRAZER_B", [0, 83, 10], [100, 83, 10], 20, 1, 5, 0))

    sim.add_vehicle(P.Vehicle("MASSIVE", [0, 300, 10], [100, 300, 10], 30, 1_000_000, 5, 0))
    sim.add_vehicle(P.Vehicle("LIGHT", [100, 300, 10], [0, 300, 10], 30, 1e-6, 5, 0))

    return sim


# ============================================================
# Random scenario generators
# ============================================================

def _rand_point(rng, low=-100.0, high=100.0, z=10.0):
    return [
        rng.uniform(low, high),
        rng.uniform(low, high),
        z
    ]


def case_random_mixed(P, seed):
    rng = random.Random(seed)
    sim = P.Simulation(astar_frame_size=8.0, allow_3d=False)

    n = rng.randint(2, 10)

    for i in range(n):
        start = _rand_point(rng)
        end = _rand_point(rng)
        speed = rng.uniform(5, 35)
        priority = 10 ** rng.uniform(-2, 3)
        margin = rng.uniform(3, 10)
        start_time = rng.choice([
            0.0, 0.0, 0.0, rng.uniform(0, 10)
        ])

        sim.add_vehicle(P.Vehicle(
            f"R{i:02d}",
            start,
            end,
            speed,
            priority,
            margin,
            start_time
        ))

    for j in range(rng.randint(0, 4)):
        x = rng.uniform(-50, 50)
        y = rng.uniform(-50, 50)
        radius = rng.uniform(5, 18)
        sim.add_obstacle(
            P.CylinderObstacle(x, y, radius, 0, 20)
        )

    return sim


def case_random_crossing(P, seed):
    rng = random.Random(seed)
    sim = P.Simulation(astar_frame_size=8.0, allow_3d=False)

    n = rng.randint(4, 10)

    for i in range(n):
        side = i % 4
        offset = rng.uniform(-65, 65)

        if side == 0:
            start = [-100, offset, 10]
            end = [100, rng.uniform(-20, 20), 10]
        elif side == 1:
            start = [100, offset, 10]
            end = [-100, rng.uniform(-20, 20), 10]
        elif side == 2:
            start = [offset, -100, 10]
            end = [rng.uniform(-20, 20), 100, 10]
        else:
            start = [offset, 100, 10]
            end = [rng.uniform(-20, 20), -100, 10]

        sim.add_vehicle(P.Vehicle(
            f"X{i:02d}",
            start,
            end,
            rng.uniform(8, 30),
            10 ** rng.uniform(-1, 2),
            rng.uniform(3.5, 8),
            rng.choice([0.0, 0.0, rng.uniform(0, 8)])
        ))

    return sim


def case_random_convergence(P, seed):
    rng = random.Random(seed)
    sim = P.Simulation(astar_frame_size=8.0, allow_3d=False)

    n = rng.randint(5, 12)

    for i in range(n):
        angle = (
            2 * math.pi * i / n
            + rng.uniform(-0.08, 0.08)
        )
        radius = rng.uniform(60, 100)

        start = [
            radius * math.cos(angle),
            radius * math.sin(angle),
            10
        ]

        end = [
            rng.uniform(-8, 8),
            rng.uniform(-8, 8),
            10
        ]

        sim.add_vehicle(P.Vehicle(
            f"C{i:02d}",
            start,
            end,
            rng.uniform(8, 28),
            10 ** rng.uniform(-2, 3),
            rng.uniform(3, 8),
            rng.choice([0.0, 0.0, rng.uniform(0, 5)])
        ))

    return sim


# ============================================================
# Deterministic suite
# ============================================================

def deterministic_suite(P):
    return [
        TestCase("01_no_collision", "Well-separated parallel traffic", case_no_collision),
        TestCase("02_head_on", "Two-vehicle head-on conflict", case_head_on),
        TestCase("03_equal_head_on", "Equal-momentum head-on conflict", case_equal_head_on),
        TestCase("04_three_way_cross", "Three-way crossing conflict", case_three_way_cross),
        TestCase("05_four_way_cross", "Four-way intersection conflict", case_four_way_cross),
        TestCase("06_diagonal_cross", "Two diagonal trajectories", case_diagonal_cross),
        TestCase("07_momentum_extreme", "Extreme priority/momentum ratio", case_momentum_extreme),
        TestCase("08_speed_extreme", "Extreme velocity case", case_speed_extreme),
        TestCase("09_staggered_injection", "Vehicles entering at different times", case_staggered_injection),
        TestCase("10_chain", "Dense chain of vehicles", case_chain),
        TestCase("11_pbd_cluster", "Severely overlapping PBD cluster", case_pbd_cluster),
        TestCase("12_zero_distance", "Identical starting positions", case_zero_distance),
        TestCase("13_obstacle_field", "Multiple static obstacles with traffic", case_obstacle_field),
        TestCase("14_inside_obstacle", "Vehicle starts inside obstacle", case_inside_obstacle),
        TestCase("15_bad_goal", "Very short/edge-case goal", case_bad_goal),
        TestCase("16_grazing", "Near-parallel close traffic", case_grazing),
        TestCase("17_vertical_2d", "Different start/end altitude with 2D mode", case_vertical_2d),
        TestCase("18_low_high", "Different-altitude opposing traffic", case_low_high),
        TestCase("19_parallel_close", "Multiple closely spaced parallel vehicles", case_parallel_close),
        TestCase("20_tight_corridor", "Traffic through constrained obstacle corridor", case_tight_corridor),
        TestCase("21_late_cross", "Late-arriving crossing traffic", case_late_cross),
        TestCase("22_swapping", "High-speed swapping trajectories", case_swapping),
        TestCase("23_original_manifest", "Full original multi-scenario manifest", case_original_manifest),
    ]


# ============================================================
# Serialization
# ============================================================

def arr3(x):
    a = np.asarray(x, dtype=float)
    return [float(v) for v in a.tolist()]


def vehicle_manifest(v):
    data = {
        "id": v.v_id,
        "start": arr3(v.start),
        "end": arr3(v.end),
        "speed": float(v.speed),
        "priority": float(v.priority),
        "momentum": float(v.momentum),
        "margin": float(v.margin),
        "start_time": float(v.start_time),
    }

    if hasattr(v, "locked_altitude"):
        data["locked_altitude"] = float(v.locked_altitude)

    return data


def obstacle_manifest(o):
    return {
        "x": float(o.x),
        "y": float(o.y),
        "radius": float(o.radius),
        "z_min": float(o.z_min),
        "z_max": float(o.z_max),
    }


def save_resolved_csv(sim, path):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "Vehicle_ID", "Margin", "Time", "X", "Y", "Z"
        ])

        for v in sim.vehicles:
            for wp in v.resolved_path:
                p = np.asarray(wp["pos"], dtype=float)

                w.writerow([
                    v.v_id,
                    float(v.margin),
                    round(float(wp["t"]), 2),
                    round(float(p[0]), 2),
                    round(float(p[1]), 2),
                    round(float(p[2]), 2),
                ])


def save_original_csv(sim, path):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "Vehicle_ID", "Time", "X", "Y", "Z"
        ])

        for v in sim.vehicles:
            for wp in v.original_path:
                p = np.asarray(wp["pos"], dtype=float)

                w.writerow([
                    v.v_id,
                    round(float(wp["t"]), 2),
                    round(float(p[0]), 2),
                    round(float(p[1]), 2),
                    round(float(p[2]), 2),
                ])


# ============================================================
# Safety checks
# ============================================================

def build_path_lookup(sim):
    return {
        v.v_id: {
            round(float(wp["t"]), 2): np.asarray(
                wp["pos"], dtype=float
            )
            for wp in v.resolved_path
        }
        for v in sim.vehicles
    }


def effective_goal(v, sim):
    if not sim.allow_3d and hasattr(v, "locked_altitude"):
        goal = np.copy(v.end)
        goal[2] = v.locked_altitude
        return goal

    return v.end


def current_safety_check(sim):
    paths = build_path_lookup(sim)

    timestamps = sorted(set(
        round(float(wp["t"]), 2)
        for v in sim.vehicles
        for wp in v.resolved_path
    ))

    violations = []
    min_pair_distance = math.inf

    for t in timestamps:
        active_at_t = []

        for v in sim.vehicles:
            if (
                t > float(v.start_time)
                and (
                    float(v.new_time) == 0.0
                    or t <= float(v.new_time)
                )
            ):
                pos = paths[v.v_id].get(t)

                if pos is None:
                    continue

                if np.linalg.norm(
                    pos - effective_goal(v, sim)
                ) > 0.5:
                    active_at_t.append((v, pos))

        for i in range(len(active_at_t)):
            for j in range(i + 1, len(active_at_t)):
                v1, p1 = active_at_t[i]
                v2, p2 = active_at_t[j]

                dist = float(np.linalg.norm(p1 - p2))
                min_pair_distance = min(
                    min_pair_distance,
                    dist
                )

                min_allowed = float(
                    v1.margin + v2.margin - 0.05
                )

                if dist < min_allowed:
                    violations.append({
                        "time": float(t),
                        "vehicle_a": v1.v_id,
                        "vehicle_b": v2.v_id,
                        "distance": dist,
                        "allowed": float(
                            v1.margin + v2.margin
                        ),
                        "threshold_used_by_current_check": min_allowed,
                        "position_a": arr3(p1),
                        "position_b": arr3(p2),
                    })

    return (
        violations,
        None if math.isinf(min_pair_distance)
        else min_pair_distance
    )


def goal_check(sim):
    vehicles = []

    for v in sim.vehicles:
        final_pos = np.asarray(
            v.resolved_path[-1]["pos"],
            dtype=float
        )

        goal = effective_goal(v, sim)
        dist_to_goal = float(
            np.linalg.norm(final_pos - goal)
        )

        reached = dist_to_goal <= 0.5

        if reached:
            actual_duration = (
                float(v.new_time - v.start_time)
                if v.new_time
                else None
            )

            delay = (
                max(0.0, actual_duration - float(v.orig_time))
                if actual_duration is not None
                else None
            )
        else:
            actual_duration = None
            delay = None

        vehicles.append({
            "vehicle": v.v_id,
            "reached_goal": reached,
            "final_distance_to_goal": dist_to_goal,
            "ideal_time": float(v.orig_time),
            "actual_time": actual_duration,
            "delay": delay,
        })

    return vehicles


def obstacle_clearance_check(sim):
    min_clearance = math.inf
    obstacle_violations = []

    for v in sim.vehicles:
        for wp in v.resolved_path:
            p = np.asarray(wp["pos"], dtype=float)

            for obs in sim.obstacles:
                if obs.z_min <= p[2] <= obs.z_max:
                    d = float(np.linalg.norm(
                        p[:2]
                        - np.array([obs.x, obs.y], dtype=float)
                    ))

                    required = float(
                        obs.radius + v.margin
                    )

                    clearance = d - required
                    min_clearance = min(
                        min_clearance,
                        clearance
                    )

                    if clearance < 0:
                        obstacle_violations.append({
                            "time": float(wp["t"]),
                            "vehicle": v.v_id,
                            "clearance": clearance,
                            "distance_to_center": d,
                            "required_center_distance": required,
                            "position": arr3(p),
                        })

    return (
        obstacle_violations,
        None if math.isinf(min_clearance)
        else min_clearance
    )


def path_length(points):
    if len(points) < 2:
        return 0.0

    arr = np.asarray(points, dtype=float)

    return float(
        np.sum(np.linalg.norm(
            np.diff(arr, axis=0),
            axis=1
        ))
    )


def motion_metrics(sim):
    rows = []
    total_ideal = 0.0
    total_actual = 0.0
    total_delay = 0.0
    reached_count = 0

    for v in sim.vehicles:
        resolved_points = [
            wp["pos"] for wp in v.resolved_path
        ]
        original_points = [
            wp["pos"] for wp in v.original_path
        ]

        actual = None
        delay = None

        if v.new_time:
            actual = float(
                v.new_time - v.start_time
            )
            delay = max(
                0.0,
                actual - float(v.orig_time)
            )

        if delay is not None:
            total_actual += actual
            total_delay += delay

        total_ideal += float(v.orig_time)

        if np.linalg.norm(
            np.asarray(v.resolved_path[-1]["pos"])
            - effective_goal(v, sim)
        ) <= 0.5:
            reached_count += 1

        rows.append({
            "vehicle": v.v_id,
            "ideal_time_s": float(v.orig_time),
            "actual_time_s": actual,
            "delay_s": delay,
            "original_path_length_m": path_length(
                original_points
            ),
            "resolved_path_length_m": path_length(
                resolved_points
            ),
            "start_time_s": float(v.start_time),
            "speed": float(v.speed),
            "priority": float(v.priority),
            "momentum": float(v.momentum),
            "margin": float(v.margin),
            "steering_events": sum(
                1
                for e in sim.deflection_events
                if e["v_id"] == v.v_id
            ),
            "state_changes_recorded": max(
                0,
                len(v.state_history) - 1
            ),
        })

    return rows, {
        "total_ideal_time_sum_s": total_ideal,
        "total_actual_time_sum_s": (
            total_actual
            if reached_count == len(sim.vehicles)
            else None
        ),
        "total_delay_sum_s": (
            total_delay
            if reached_count == len(sim.vehicles)
            else None
        ),
        "average_delay_s": (
            total_delay / len(sim.vehicles)
            if reached_count == len(sim.vehicles)
            and sim.vehicles
            else None
        ),
        "vehicles_reached": reached_count,
        "vehicle_count": len(sim.vehicles),
    }


# ============================================================
# Capture original verifier output
# ============================================================

def run_and_capture(sim, case_dir):
    original_verify = sim.verify_manifest
    captured = io.StringIO()

    def wrapped_verify():
        with contextlib.redirect_stdout(captured):
            original_verify()

    def wrapped_export():
        save_resolved_csv(
            sim,
            case_dir / "resolved_paths.csv"
        )

    sim.verify_manifest = wrapped_verify
    sim.export_csv = wrapped_export

    error = None

    try:
        sim.run()
    except Exception as exc:
        error = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }

    report = captured.getvalue()

    (case_dir / "safety_report.txt").write_text(
        report,
        encoding="utf-8"
    )

    return error


# ============================================================
# Plot helpers
# ============================================================

def safe_filename(name):
    allowed = set(
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789"
        "-_."
    )

    cleaned = "".join(
        c if c in allowed else "_"
        for c in name
    )

    return cleaned.strip("._") or "scenario"


def save_final_plot(sim, result, case_dir):
    try:
        import plotly.graph_objects as go
    except Exception as exc:
        (case_dir / "plot_error.txt").write_text(
            f"Plotly unavailable: "
            f"{type(exc).__name__}: {exc}\n",
            encoding="utf-8"
        )
        return None

    scenario_name = safe_filename(
        result["summary"]["case_name"]
    )

    fig = go.Figure()

    # Obstacles
    for obs in sim.obstacles:
        z = np.linspace(
            obs.z_min,
            obs.z_max,
            12
        )

        theta = np.linspace(
            0,
            2 * np.pi,
            40
        )

        tg, zg = np.meshgrid(theta, z)

        xg = obs.radius * np.cos(tg) + obs.x
        yg = obs.radius * np.sin(tg) + obs.y

        fig.add_trace(go.Surface(
            x=xg,
            y=yg,
            z=zg,
            colorscale="Greys",
            opacity=0.75,
            showscale=False,
            hoverinfo="skip",
            name="Obstacle"
        ))

    palette = [
        "#1f77b4", "#ff7f0e", "#2ca02c",
        "#d62728", "#9467bd", "#8c564b",
        "#e377c2", "#17becf", "#bcbd22"
    ]

    for idx, v in enumerate(sim.vehicles):
        c = palette[idx % len(palette)]

        if v.original_path:
            orig = np.asarray([
                wp["pos"] for wp in v.original_path
            ], dtype=float)

            fig.add_trace(go.Scatter3d(
                x=orig[:, 0],
                y=orig[:, 1],
                z=orig[:, 2],
                mode="lines",
                line=dict(
                    color=c,
                    dash="dash",
                    width=2
                ),
                opacity=0.28,
                name=f"{v.v_id} original",
                legendgroup=v.v_id,
                showlegend=False,
                hoverinfo="skip"
            ))

        resolved = np.asarray([
            wp["pos"] for wp in v.resolved_path
        ], dtype=float)

        delay = next(
            (
                x["delay_s"]
                for x in result["vehicle_metrics"]
                if x["vehicle"] == v.v_id
            ),
            None
        )

        actual = next(
            (
                x["actual_time_s"]
                for x in result["vehicle_metrics"]
                if x["vehicle"] == v.v_id
            ),
            None
        )

        if actual is not None:
            hover = (
                f"{v.v_id}"
                f"<br>Ideal time: {v.orig_time:.2f}s"
                f"<br>Actual time: {actual:.2f}s"
            )
        else:
            hover = (
                f"{v.v_id}"
                f"<br>Ideal time: {v.orig_time:.2f}s"
                f"<br>Actual time: not reached"
            )

        if delay is not None:
            hover += f"<br>Delay: {delay:.2f}s"

        hover += "<br>Resolved trajectory"

        fig.add_trace(go.Scatter3d(
            x=resolved[:, 0],
            y=resolved[:, 1],
            z=resolved[:, 2],
            mode="lines",
            line=dict(color=c, width=4),
            name=f"{v.v_id} resolved",
            legendgroup=v.v_id,
            hovertemplate=hover + "<extra></extra>"
        ))

        fig.add_trace(go.Scatter3d(
            x=[v.start[0]],
            y=[v.start[1]],
            z=[v.start[2]],
            mode="markers",
            marker=dict(
                size=6,
                color="lightgreen"
            ),
            name=f"{v.v_id} start",
            legendgroup=v.v_id,
            showlegend=False,
            hovertemplate=(
                f"{v.v_id} start<extra></extra>"
            )
        ))

        fig.add_trace(go.Scatter3d(
            x=[v.end[0]],
            y=[v.end[1]],
            z=[v.end[2]],
            mode="markers",
            marker=dict(
                size=6,
                color="red",
                symbol="x"
            ),
            name=f"{v.v_id} goal",
            legendgroup=v.v_id,
            showlegend=False,
            hovertemplate=(
                f"{v.v_id} goal<extra></extra>"
            )
        ))

    # Steering events
    if sim.deflection_events:
        dpts = np.asarray([
            e["pos"] for e in sim.deflection_events
        ], dtype=float)

        dlabels = [
            f"Steering: {e['v_id']}"
            for e in sim.deflection_events
        ]

        fig.add_trace(go.Scatter3d(
            x=dpts[:, 0],
            y=dpts[:, 1],
            z=dpts[:, 2],
            mode="markers",
            marker=dict(
                size=5,
                color="yellow",
                symbol="diamond"
            ),
            text=dlabels,
            hovertemplate="%{text}<extra></extra>",
            name="Steering events"
        ))

    violations = result["safety"]["center_to_center"]["violations"]

    if violations:
        xs, ys, zs = [], [], []
        labels = []

        for viol in violations:
            for p in (
                viol["position_a"],
                viol["position_b"]
            ):
                xs.append(p[0])
                ys.append(p[1])
                zs.append(p[2])

                labels.append(
                    f"Violation @ "
                    f"{viol['time']:.2f}s: "
                    f"{viol['vehicle_a']} / "
                    f"{viol['vehicle_b']}<br>"
                    f"Distance={viol['distance']:.2f}m; "
                    f"Allowed={viol['allowed']:.2f}m"
                )

        fig.add_trace(go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="markers",
            marker=dict(
                size=8,
                color="black",
                symbol="x"
            ),
            text=labels,
            hovertemplate="%{text}<extra></extra>",
            name="Collision / safety violations"
        ))

    summary = result["summary"]

    if summary["average_delay_s"] is not None:
        title = (
            f"{summary['case_name']} | "
            f"Goals: "
            f"{'PASS' if summary['all_goals_reached'] else 'FAIL'} | "
            f"Safety: "
            f"{'PASS' if summary['center_to_center_violations'] == 0 else 'FAIL'} | "
            f"Violations: "
            f"{summary['center_to_center_violations']} | "
            f"Steering events: "
            f"{summary['steering_events']} | "
            f"Delay: "
            f"{summary['average_delay_s']:.2f}s"
        )
    else:
        title = (
            f"{summary['case_name']} | "
            f"Goals: "
            f"{'PASS' if summary['all_goals_reached'] else 'FAIL'} | "
            f"Safety: "
            f"{'PASS' if summary['center_to_center_violations'] == 0 else 'FAIL'} | "
            f"Violations: "
            f"{summary['center_to_center_violations']} | "
            f"Steering events: "
            f"{summary['steering_events']}"
        )

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(title="X"),
            yaxis=dict(title="Y"),
            zaxis=dict(title="Z"),
            aspectmode="data"
        ),
        legend=dict(
            x=1.02,
            y=1
        ),
        margin=dict(
            l=0,
            r=230,
            b=0,
            t=85
        )
    )

    # ========================================================
    # REQUESTED CHANGE:
    # Use scenario name as the actual plot filename.
    # ========================================================
    html_path = case_dir / f"{scenario_name}.html"
    png_path = case_dir / f"{scenario_name}.png"

    fig.write_html(
        str(html_path),
        include_plotlyjs=True
    )

    try:
        fig.write_image(
            str(png_path),
            width=1800,
            height=1100,
            scale=1
        )

        return {
            "html": html_path.name,
            "png": png_path.name
        }

    except Exception as plotly_png_exc:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            mfig = plt.figure(
                figsize=(16, 10)
            )

            ax = mfig.add_subplot(
                111,
                projection="3d"
            )

            for idx, v in enumerate(sim.vehicles):
                c = palette[idx % len(palette)]

                if v.original_path:
                    op = np.asarray([
                        wp["pos"]
                        for wp in v.original_path
                    ], dtype=float)

                    ax.plot(
                        op[:, 0],
                        op[:, 1],
                        op[:, 2],
                        linestyle="--",
                        linewidth=1.2,
                        alpha=0.28,
                        color=c
                    )

                rp = np.asarray([
                    wp["pos"]
                    for wp in v.resolved_path
                ], dtype=float)

                ax.plot(
                    rp[:, 0],
                    rp[:, 1],
                    rp[:, 2],
                    linewidth=2.2,
                    color=c,
                    label=f"{v.v_id} resolved"
                )

                ax.scatter(
                    [v.start[0]],
                    [v.start[1]],
                    [v.start[2]],
                    s=35,
                    marker="o",
                    color="green"
                )

                ax.scatter(
                    [v.end[0]],
                    [v.end[1]],
                    [v.end[2]],
                    s=45,
                    marker="x",
                    color="red"
                )

            if sim.deflection_events:
                dp = np.asarray([
                    e["pos"]
                    for e in sim.deflection_events
                ], dtype=float)

                ax.scatter(
                    dp[:, 0],
                    dp[:, 1],
                    dp[:, 2],
                    s=24,
                    marker="D",
                    color="gold",
                    label="Steering events"
                )

            if violations:
                vp = []

                for viol in violations:
                    vp.append(viol["position_a"])
                    vp.append(viol["position_b"])

                vp = np.asarray(vp, dtype=float)

                ax.scatter(
                    vp[:, 0],
                    vp[:, 1],
                    vp[:, 2],
                    s=48,
                    marker="x",
                    color="black",
                    label="Safety violations"
                )

            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(title)

            ax.legend(
                loc="upper left",
                bbox_to_anchor=(1.02, 1.0),
                fontsize=7
            )

            mfig.tight_layout()
            mfig.savefig(
                png_path,
                dpi=160,
                bbox_inches="tight"
            )

            plt.close(mfig)

            (case_dir / "plot_png_fallback.txt").write_text(
                "Plotly PNG export was unavailable "
                f"({type(plotly_png_exc).__name__}: "
                f"{plotly_png_exc}). "
                "Static PNG was generated with matplotlib.\n",
                encoding="utf-8"
            )

            return {
                "html": html_path.name,
                "png": png_path.name
            }

        except Exception as mpl_exc:
            (case_dir / "plot_png_error.txt").write_text(
                "Plotly PNG export failed: "
                f"{type(plotly_png_exc).__name__}: "
                f"{plotly_png_exc}\n"
                "Matplotlib fallback also failed: "
                f"{type(mpl_exc).__name__}: "
                f"{mpl_exc}\n",
                encoding="utf-8"
            )

            return {
                "html": html_path.name,
                "png": None
            }


# ============================================================
# One test case
# ============================================================

def run_case(P, test, root):
    case_dir = root / test.name

    if case_dir.exists():
        shutil.rmtree(case_dir)

    case_dir.mkdir(parents=True)

    sim = test.builder(P)

    manifest = {
        "case_name": test.name,
        "description": test.description,
        "category": test.category,
        "seed": test.seed,
        "simulation": {
            "dt_s": float(sim.dt),
            "astar_frame_size": float(
                sim.astar_frame_size
            ),
            "allow_3d": bool(sim.allow_3d),
        },
        "vehicles": [
            vehicle_manifest(v)
            for v in sim.vehicles
        ],
        "obstacles": [
            obstacle_manifest(o)
            for o in sim.obstacles
        ],
    }

    (case_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8"
    )

    error = run_and_capture(
        sim,
        case_dir
    )

    save_original_csv(
        sim,
        case_dir / "original_paths.csv"
    )

    if not (case_dir / "resolved_paths.csv").exists():
        save_resolved_csv(
            sim,
            case_dir / "resolved_paths.csv"
        )

    serial_events = [
        {
            "v_id": e["v_id"],
            "pos": arr3(e["pos"])
        }
        for e in sim.deflection_events
    ]

    (case_dir / "deflection_events.json").write_text(
        json.dumps(serial_events, indent=2),
        encoding="utf-8"
    )

    goals = goal_check(sim)

    violations, min_pair_distance = (
        current_safety_check(sim)
    )

    obs_violations, min_obs_clearance = (
        obstacle_clearance_check(sim)
    )

    vehicle_metrics, aggregate = (
        motion_metrics(sim)
    )

    all_goals_reached = all(
        x["reached_goal"]
        for x in goals
    )

    safety_pass = len(violations) == 0

    summary = {
        "case_name": test.name,
        "category": test.category,
        "seed": test.seed,
        "simulation_error": error is not None,
        "all_goals_reached": all_goals_reached,
        "center_to_center_violations": len(violations),
        "obstacle_penetration_violations": len(obs_violations),
        "current_safety_checks_pass": (
            all_goals_reached
            and safety_pass
        ),
        "strict_diagnostic_pass": (
            all_goals_reached
            and safety_pass
            and len(obs_violations) == 0
        ),
        "steering_events": len(
            sim.deflection_events
        ),
        "min_center_to_center_distance_m": (
            min_pair_distance
        ),
        "min_obstacle_clearance_m": (
            min_obs_clearance
        ),
        "average_delay_s": (
            aggregate["average_delay_s"]
        ),
        "total_delay_sum_s": (
            aggregate["total_delay_sum_s"]
        ),
        "vehicles_reached": (
            aggregate["vehicles_reached"]
        ),
        "vehicle_count": (
            aggregate["vehicle_count"]
        ),
        "dt_s": float(sim.dt),
        "allow_3d": bool(sim.allow_3d),
    }

    result = {
        "summary": summary,
        "goals": goals,
        "vehicle_metrics": vehicle_metrics,
        "safety": {
            "center_to_center": {
                "violations": violations,
                "count": len(violations),
                "minimum_distance_m": min_pair_distance,
                "rule": (
                    "distance < "
                    "(margin_i + margin_j - 0.05)"
                ),
            },
            "obstacle_diagnostic": {
                "violations": obs_violations,
                "count": len(obs_violations),
                "minimum_clearance_m": min_obs_clearance,
                "rule": (
                    "distance_to_obstacle_center - "
                    "(radius + vehicle_margin) < 0"
                ),
            },
            "goals": goals,
        },
        "original_verify_manifest_output": (
            "safety_report.txt"
        ),
    }

    (case_dir / "result.json").write_text(
        json.dumps(result, indent=2),
        encoding="utf-8"
    )

    with (case_dir / "metrics.csv").open(
        "w",
        newline=""
    ) as f:
        fieldnames = (
            list(vehicle_metrics[0].keys())
            if vehicle_metrics
            else ["vehicle"]
        )

        w = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        w.writeheader()
        w.writerows(vehicle_metrics)

    plot_info = save_final_plot(
        sim,
        result,
        case_dir
    )

    result["plot"] = plot_info

    (case_dir / "result.json").write_text(
        json.dumps(result, indent=2),
        encoding="utf-8"
    )

    return result


# ============================================================
# Main runner
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Autonomous PHASE test suite"
    )

    parser.add_argument(
        "--phase-file",
        default=str(
            Path(__file__).with_name(
                "phase_altitude_locked.py"
            )
        ),
        help="Path to PHASE Python source under test."
    )

    parser.add_argument(
        "--output",
        default="phase_test_results",
        help="Directory for test outputs."
    )

    parser.add_argument(
        "--random",
        type=int,
        default=10,
        help=(
            "Number of random scenarios per random "
            "family (default: 10)."
        )
    )

    parser.add_argument(
        "--cases",
        default="all",
        help=(
            "Comma-separated deterministic case "
            "names, or 'all'."
        )
    )

    parser.add_argument(
        "--skip-original",
        action="store_true",
        help=(
            "Skip the full original multi-scenario "
            "manifest case."
        )
    )

    args = parser.parse_args()

    phase_path = Path(
        args.phase_file
    ).resolve()

    if not phase_path.exists():
        raise FileNotFoundError(
            f"PHASE file not found: {phase_path}"
        )

    output_root = Path(
        args.output
    ).resolve()

    output_root.mkdir(
        parents=True,
        exist_ok=True
    )

    source_hash = hashlib.sha256(
        phase_path.read_bytes()
    ).hexdigest()

    source_info = {
        "phase_source": str(phase_path),
        "sha256": source_hash,
        "python": sys.version,
        "numpy": np.__version__,
    }

    (output_root / "source_info.json").write_text(
        json.dumps(source_info, indent=2),
        encoding="utf-8"
    )

    P = load_module_from_file(
        phase_path
    )

    tests = deterministic_suite(P)

    if args.cases != "all":
        wanted = {
            x.strip()
            for x in args.cases.split(",")
            if x.strip()
        }

        tests = [
            t for t in tests
            if t.name in wanted
        ]

    if args.skip_original:
        tests = [
            t for t in tests
            if t.name != "23_original_manifest"
        ]

    random_count = max(
        0,
        args.random
    )

    for i in range(random_count):
        seed = 100000 + i

        tests.append(TestCase(
            f"R01_mixed_{i:04d}",
            "Seeded randomized mixed traffic and obstacles",
            lambda P, s=seed: case_random_mixed(P, s),
            category="random_mixed",
            seed=seed,
        ))

        tests.append(TestCase(
            f"R02_crossing_{i:04d}",
            "Seeded randomized crossing traffic",
            lambda P, s=seed + 10000: case_random_crossing(P, s),
            category="random_crossing",
            seed=seed + 10000,
        ))

        tests.append(TestCase(
            f"R03_convergence_{i:04d}",
            "Seeded randomized converging traffic",
            lambda P, s=seed + 20000: case_random_convergence(P, s),
            category="random_convergence",
            seed=seed + 20000,
        ))

    all_results = []

    print("=" * 92)
    print("PHASE AUTONOMOUS TEST SUITE")
    print("=" * 92)
    print(f"PHASE source : {phase_path}")
    print(f"SHA-256      : {source_hash}")
    print(f"Test cases   : {len(tests)}")
    print(f"Output       : {output_root}")
    print("=" * 92)

    for index, test in enumerate(
        tests,
        start=1
    ):
        print(
            f"[{index:>3}/{len(tests)}] "
            f"{test.name:<28}",
            end="",
            flush=True
        )

        try:
            result = run_case(
                P,
                test,
                output_root
            )

            s = result["summary"]

            status = (
                "PASS"
                if s["current_safety_checks_pass"]
                else "FAIL"
            )

            extra = (
                f" | goals "
                f"{s['vehicles_reached']}/"
                f"{s['vehicle_count']}"
                f" | safety violations "
                f"{s['center_to_center_violations']}"
                f" | steering "
                f"{s['steering_events']}"
            )

            print(
                f"{status}{extra}"
            )

            all_results.append(
                result
            )

        except Exception as exc:
            print(
                f"ERROR: "
                f"{type(exc).__name__}: {exc}"
            )

            all_results.append({
                "summary": {
                    "case_name": test.name,
                    "category": test.category,
                    "seed": test.seed,
                    "current_safety_checks_pass": False,
                    "strict_diagnostic_pass": False,
                    "all_goals_reached": False,
                    "simulation_error": True,
                    "vehicles_reached": 0,
                    "vehicle_count": 0,
                    "center_to_center_violations": None,
                    "obstacle_penetration_violations": None,
                    "steering_events": None,
                    "min_center_to_center_distance_m": None,
                    "min_obstacle_clearance_m": None,
                    "average_delay_s": None,
                },
                "fatal_error": {
                    "type": type(exc).__name__,
                    "message": str(exc),
                    "traceback": traceback.format_exc(),
                },
            })

    fieldnames = [
        "case_name",
        "category",
        "seed",
        "current_safety_checks_pass",
        "strict_diagnostic_pass",
        "all_goals_reached",
        "vehicles_reached",
        "vehicle_count",
        "center_to_center_violations",
        "obstacle_penetration_violations",
        "steering_events",
        "min_center_to_center_distance_m",
        "min_obstacle_clearance_m",
        "average_delay_s",
        "simulation_error"
    ]

    with (
        output_root / "run_summary.csv"
    ).open(
        "w",
        newline=""
    ) as f:
        w = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        w.writeheader()

        for r in all_results:
            w.writerow({
                k: r["summary"].get(k)
                for k in fieldnames
            })

    run_summary = {
        "source": source_info,
        "test_count": len(all_results),
        "current_safety_passes": sum(
            1
            for r in all_results
            if r.get("summary", {}).get(
                "current_safety_checks_pass"
            )
        ),
        "current_safety_failures": sum(
            1
            for r in all_results
            if not r.get("summary", {}).get(
                "current_safety_checks_pass"
            )
        ),
        "strict_diagnostic_passes": sum(
            1
            for r in all_results
            if r.get("summary", {}).get(
                "strict_diagnostic_pass"
            )
        ),
        "results": all_results,
    }

    (output_root / "run_summary.json").write_text(
        json.dumps(
            run_summary,
            indent=2
        ),
        encoding="utf-8"
    )

    print("=" * 92)
    print("TEST COMPLETE")
    print(
        f"Current safety PASS: "
        f"{run_summary['current_safety_passes']}/"
        f"{len(all_results)}"
    )
    print(
        f"Current safety FAIL: "
        f"{run_summary['current_safety_failures']}/"
        f"{len(all_results)}"
    )
    print(
        f"Strict diagnostic PASS: "
        f"{run_summary['strict_diagnostic_passes']}/"
        f"{len(all_results)}"
    )
    print(f"Results: {output_root}")
    print("=" * 92)


if __name__ == "__main__":
    main()
