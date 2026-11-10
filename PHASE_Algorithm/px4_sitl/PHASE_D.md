# PHASE-D: dynamics-aware PHASE (experimental)

**Status:** experimental variant, tested on 3 PX4 SIH quadcopters. Implemented in
`phase_dynamic.py`. PHASE V1.2 itself is unchanged; PHASE-D reuses its `Vehicle`,
`FramedAStarEngine`, momentum model, deflection-distance rule and PBD.
Change history: `CHANGELOG.md` (IDs D-01 … D-12; harness H-01 … H-11).

## 1. Why it exists

PHASE models vehicles like billiard balls. Real billiard collisions resolve with
an instantaneous impulse; quadcopters can't change velocity faster than about
3 m/s². On PX4, V1.2's *commanded* positions always kept the 5.2 m margin, but the
*real* drones overshot to 1.79–4.21 m (CHANGELOG F-01). PHASE-D keeps PHASE's
priority/momentum idea and adds the vehicle's physical limits.

## 2. Rule-by-rule difference from V1.2

| # | Rule | V1.2 | PHASE-D |
|---|---|---|---|
| 1 | Collision volume | Fixed: margin + 0.2 m buffer | **Growing ball**: margin + reaction distance + braking distance, along the approach direction only (§4.1) |
| 2 | Conflict trigger | Proposed positions one tick ahead overlap | Balls touch at current positions (§4.1) |
| 3 | Who slows down | Nobody; position jumps via PBD | **Priority braking**: lower momentum yields first, higher only for what the lower can't remove; obstacles never yield (§4.3) |
| 4 | Command limits | None (teleporting positions) | Speed ≤ vehicle speed, acceleration ≤ `a_cmd`; braking gets the acceleration budget first (§4.5) |
| 5 | Vehicle lag | Not modelled | `tau_response` sizes balls and inflates braking (§4.2) |
| 6 | Deflection direction | 90° left/right, scored against **one** partner | Same 90° candidates and 3:1 scoring, against **all vehicles in the planning frame** (§4.4) |
| 7 | Deflection distance | max(share × speed × 4, 1.5 × margin) | Unchanged |
| 8 | Deflection lifetime | One tick (global replan resets it) | Runs until arrived / stalled 1 s / time-out (§4.6) |
| 9 | Re-deflection | Only from NORMAL, only on new pairs | Same rule, also while deflecting and when a deflection ends inside an active conflict |
| 10 | Replan on new conflict | Every active vehicle | Only NORMAL vehicles within the planning frame of the conflict |
| 11 | Neighbour search | All pairs, O(N²) | Spatial hash grid, O(N·k) (§5) |
| 12 | PBD | Unbounded position correction | Kept as last resort, but its velocity is bounded; any excess is counted as `pbd_unrealizable` |
| 13 | Equal-momentum tie-break | 1.5× boost for the first vehicle | Unchanged, also used for braking precedence |
| 14 | Waypoint advance | Within 0.5 m (exact steps, so always hit) | Within max(0.5 m, ‖v‖·dt) **or once passed** (beyond it on the way to the next), so an acceleration-limited vehicle can't orbit a waypoint (D-11) |

## 3. Per-tick pipeline (dt = 0.2 s)

1. **State.** Activation and goal check. Deflections are checked for completion; one that ends inside an active conflict re-deflects by the rule. NORMAL vehicles follow their A* path. Desired velocity is capped by a braking curve towards the goal.
2. **Conflicts.** Candidate pairs come from the grid. A pair whose balls newly touch triggers deflection: one per vehicle per tick, against its nearest new partner. NORMAL vehicles near that conflict replan.
3. **Priority braking.** Candidate velocity = desired, after the acceleration limit. Closing speeds are capped per pair (3 passes), then obstacles.
4. **Limits.** Commit within the acceleration and speed limits, braking first.
5. **PBD.** V1.2's correction, bounded.
6. **Commit.** As V1.2, plus the velocity state.

## 4. Equations

### 4.1 Growing ball
For vehicle *i* approaching object *j*, let u be the unit vector i→j and
cᵢ = max(0, vᵢ·u). vᵢ is the largest of the measured, commanded and desired velocity.

    Rᵢ = marginᵢ + cᵢ·(t_latency + tau) + cᵢ² / (2·a_brake)

The balls touch when ‖pⱼ − pᵢ‖ < Rᵢ + Rⱼ + 0.2. Head-on traffic grows the full
stopping distance; parallel or diverging traffic keeps the bare margin.

### 4.2 Safe closing speed and response lag
With gap g = d − (marginᵢ + marginⱼ + 0.2) and t = t_latency + tau, the largest
closing speed that can still be stopped (one vehicle braking) is

    c_safe(g) = a_brake · ( −t + √(t² + 2g / a_brake) )

A first-order vehicle whose real velocity is still ahead of its command covers
an extra (c_meas − c_cmd)⁺ · tau before it catches up, so braking uses

    g_eff = g − max(0, c_meas − c_cmd) · tau

### 4.3 Priority braking
For each pair, excess = c_cmd − c_safe(g_eff). If excess > 0:

1. The **lower-momentum** vehicle removes it from its own approach velocity. It may back away, down to −its speed.
2. Whatever is left goes to the **higher-momentum** vehicle.
3. Momentum = priority × speed, with PHASE's 1.5× tie-break.

For obstacles, the vehicle always yields (infinite obstacle momentum).

### 4.4 Deflection direction (all nearby vehicles)
The candidates are PHASE's: 90° left or right of the current heading. Evasion vector:

    e = Σₖ reachₖ · unit(pᵢ − pₖ)  +  PHASE's obstacle repulsion
    reachₖ = marginᵢ + marginₖ + 0.2 + stopping extents of i and k along the line

The sum is over every vehicle k within `astar_frame_size` (the conflict partner is
always included). Score = 3·(candidate·ê) + 1·(candidate·goal direction), as in PHASE.
With only the partner and no obstacles this is exactly PHASE's choice (tested).

### 4.5 Acceleration and speed limits with braking priority
Let b be the direction braking changed the velocity. The velocity change
Δv = v_target − v_cmd,prev is clipped to `a_cmd`·dt, giving the component along b
first and what's left to steering. The result is then clipped to the vehicle's
speed the same way.

### 4.6 Deflection completion
A deflection ends when the vehicle is within max(0.5 m, |v|·dt) of the target, or
has made no progress for 1.0 s, or has exceeded distance/speed + 3 s. If the
conflict that caused it is still active, a new deflection is computed by the
same rule. Otherwise the vehicle returns to NORMAL and replans.

## 5. Neighbour search: spatial hash grid

Checking every pair is O(N²). The grid cell is the largest distance at which two
vehicles can still interact this tick:

    2·max margin + 0.2 + stopping_extent(2·vmax) + 2·vmax·tau + 1 m

Every vehicle is hashed into its cell (O(N)), and only the 3×3 neighbouring cells
are searched, so pair finding is O(N·k). At constant density k stays constant.
`use_grid=False` keeps the O(N²) reference, and tests check that both give
identical trajectories.

Benchmark (kinematic, random fleets at 400 m² per vehicle; `bench_scaling.py`):

| vehicles | method | pair checks / tick | ms / tick (after tick 1) | first tick s (initial A*) |
|---|---|---|---|---|
| 10 | grid | 15 | 0.5 | 0.003 |
| 10 | brute force | 42 | 0.7 | 0.002 |
| 30 | grid | 80 | 2.6 | 0.014 |
| 30 | brute force | 435 | 6.8 | 0.019 |
| 100 | grid | 318 | 10.2 | 0.079 |
| 100 | brute force | 4,950 | 67.2 | 0.149 |
| 300 | grid | 948 | 44.6 | 0.414 |
| 300 | brute force | 44,850 | 589.8 | 1.079 |
| 1000 | grid | 3,248 | 208.2 | 2.566 |

Grid pair checks grow linearly, about 3.2 per vehicle. Brute force grows as N²/2:
44,850 pairs at 300 vehicles, and ~500,000 extrapolated at 1,000 (not run).
At 300 vehicles the grid tick is 13× faster.

The remaining cost is A* replanning. Before D-09 every uninvolved vehicle
replanned on any new conflict, about 0.57·N A* searches per tick. Localizing
replanning to the conflict's planning frame cut the 1,000-vehicle tick from
1,557 ms to about 200 ms. That is still above the 200 ms real-time budget for
1,000 vehicles in pure Python.

## 6. Parameters

| Parameter | Default | Source / meaning | Where to change |
|---|---|---|---|
| `a_brake` | 2.5 m/s² | Braking assumed in ball/closing-speed maths; below PX4 `MPC_ACC_HOR` = 3.0 for margin | `make_dynamic_stepper_class(a_brake=)` |
| `a_cmd` | 3.0 m/s² | Max commanded velocity change (PX4 `MPC_ACC_HOR`) | `a_cmd=` |
| `t_latency` | 0.25 s | One PHASE tick (0.2 s) + telemetry (~0.05 s) | `t_latency=`, bridge `--t-latency` |
| `tau_response` | 0.556 s | Vehicle response time constant: PX4 velocity loop 1/`MPC_XY_VEL_P_ACC` (1.8) | `tau_response=`, bridge `--tau-response` |
| `DEFLECT_STALL_S` | 1.0 s | Deflection ends after this long without progress | module constant |
| `use_grid` | True | Spatial grid vs O(N²) | `use_grid=`, bridge `--no-grid` |
| planning frame | `astar_frame_size` (15 m in the scenarios) | Neighbours for deflection scoring, A* walls, local replanning | scenario config |

## 7. Results

Worst **measured** separation (5.0 m required) and violation ticks per flight.
PHASE-D results by development stage show how each fix contributed.
Full per-run table: `runs/comparison_final.md` (`compare.py`).

| Scenario | V1.2 (2 flights) | PHASE-D D-05 (2) | PHASE-D D-09 (2) | **PHASE-D D-11, current (2)** |
|---|---|---|---|---|
| three_way_swap | 4.21 m (24) / 3.01 m (11) | 6.62 (0) / 7.58 (0) | 7.11 (0) / 7.37 (0, Q2 timed out) | **7.87 (0) / 7.34 (0)** |
| head_on_cross | 2.21 m (23) / 1.79 m (24) | 4.36 (6) / 7.46 (0) | 8.02 (0) / 7.39 (0) | **7.20 (0) / 7.63 (0)** |
| obstacle_choke | 3.12 m (24) / 3.20 m (13) | 7.27 (0) / 8.51 (0) | 7.54 (0) / 9.07 (0) | **8.92 (0) / 8.81 (0)** |
| Obstacle clearance (2.7 m req.) | 2.01 / 2.34 m | 2.49 / 2.67 m | 2.57 / 2.36 m | **2.88 / 2.84 m** |
| Mean delay (3-way / head-on / obstacle) | 18–23 / 15–20 / 32–40 s | 20–39 / 32–56 / 46–57 s | 35–63 / 56–64 / 50–64 s | **24–29 / 18–26 / 19 s** |

Current version (D-11) on PX4:
- all 6 flights complete, with **0 separation violations** and obstacle clearance at or above the 2.7 m target;
- delays comparable to V1.2;
- PBD never asked for an unflyable correction (`pbd_unrealizable_ticks` = 0).

Kinematic (no PX4): all scenarios complete, including the 3-way swap that livelocks in V1.2.
Separation 8.7–12.8 m.

Lag model (`lag_plant.py`, PX4-like tracking): separation 7.0–9.8 m, clearance 3.03 m.

Parameters for all D-11 flights: t_latency 0.25 s, tau_response 0.556 s, a_brake 2.5 m/s², a_cmd 3.0 m/s², grid on.
Each flight records them in `meta.json` → `phase_d_params`.

## 8. Known limitations

- **Re-deflection churn.** Delays are now comparable to V1.2 (D-11 removed waypoint orbiting, which had been the largest hidden cost), but each flight still has 14–24 re-deflections, about half of them ending by the stall timer. Remaining causes (analysis of run `20261110_005046_phase_d`):
  1. large balls keep pairs "in conflict" for most of an encounter;
  2. braking slows a deflecting vehicle, the 1 s stall timer ends the deflection, and the still-active conflict starts a new one;
  3. each 90° turn is relative to the *current* heading, so turns compound into zigzags;
  4. after a conflict clears, A* heads straight back through the other vehicle's area;
  5. the lowest-priority vehicle always gets the longest sidestep (13–14.5 m).
- **Obstacle constraint sits exactly at the boundary.** There is no explicit tracking-error allowance. Since D-07 and D-11 the PX4 flights meet 2.7 m (2.84–2.88 m), but with little to spare.
- **PX4 cruise speed** is 3.1–3.6 m/s against 4 m/s commanded (harness issue H-08, not root-caused).
- **Small sample:** 2 PX4 runs per scenario, with noticeable spread between them.
- **2D altitude-locked only**, 3-vehicle scenarios on PX4. Larger fleets only kinematic.

## 9. Physics note: which behaviours are "real billiards"

| Behaviour | Like real billiard balls? |
|---|---|
| Light (low-priority) vehicle takes the bigger avoidance | **Yes.** Momentum exchange along the line of centres |
| Braking along the line between vehicles, shared by momentum | **Yes.** That is the normal component of a collision impulse |
| 90° sidestep relative to the vehicle's own heading | **No.** A ball is deflected along the contact normal (line of centres), not relative to where it was heading |
| Repeated collisions with the same partner | **No.** Billiard balls separate after one impulse. Re-collision here comes from the goal-seeking A* pulling vehicles back: a ball rolling in a bowl with no damping |
| Stall → re-deflect loop | **No.** An artefact of the stall timer treating braking as "stuck" |
| Long conflicts from big balls | **Partly.** Closer to a soft repulsive field than a hard contact |

## 10. Reproduce

```bash
cd PHASE_Algorithm/px4_sitl
.venv/bin/python -m pytest -q                       # all rules, grid = brute force, lag model
.venv/bin/python bench_scaling.py                   # scaling table
VARIANT=phase_d ./run_all.sh                        # 3 scenarios on PX4 SIH
.venv/bin/python compare.py runs/<v12 run> runs/<phase_d run> ...
```
