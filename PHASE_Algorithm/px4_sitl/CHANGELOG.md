# Changelog: PHASE PX4 SITL harness and PHASE-D

Every change made while testing PHASE on PX4 quadcopters, in the order it was
made. Each entry says **what** changed, **why** (the evidence that prompted it),
**where** (files), and how it was **verified**. IDs are stable so results,
commits and discussions can refer to them.

- `H-xx`: test harness (bridge, launcher, analysis). Doesn't change PHASE's behaviour.
- `D-xx`: PHASE-D, the experimental dynamics-aware variant (`phase_dynamic.py`).
- `F-xx`: findings about PHASE itself.

The algorithm under test is **PHASE V1.2** (`PHASE_V1.2/Testing/phase_altitude_locked.py`
in the main `Autonomous_Algorithms` checkout, sha256 `804e3cdbcc1e…`). That file has
**never been modified**. PHASE-D is a separate file that reuses its pieces.

Run directories named below are under `runs/` (git-ignored; kept locally).

---

## 2026-11-09: harness and first PX4 tests of V1.2

### H-01: Closed-loop PHASE stepper
- **What:** `PhaseStepper.step(measured)` runs one iteration of V1.2's `Simulation.run()` loop, with each vehicle's `current_pos` taken from telemetry. It loads the PHASE file unchanged via `importlib` (the filename contains dots).
- **Why:** `run()` is one monolithic loop over its own point masses, so it can't drive real vehicles.
- **Files:** `phase_stepper.py`, `test_stepper_equivalence.py`
- **Verified:** with perfect tracking, `step()` reproduces `run()` exactly for all scenarios: positions to 1e-9, plus states, arrival times and deflection counts (6 tests). The first attempt failed on tick count: `run()` loops `while t < 300` with float accumulation, giving 1501 ticks. The test now uses the same condition.

### H-02: Scenarios at quad scale
- **What:** `three_way_swap`, `head_on_cross` and `obstacle_choke`, each with 3 vehicles at 4 m/s, 2.5 m margins and 10 m altitude.
- **Files:** `scenarios.py`

### H-03: PX4 SIH launcher
- **What:** one PX4 v1.17.0 SIH quadx instance per vehicle (airframe 10040), each spawned at its scenario start point via `PX4_HOME_LAT/LON`, with offboard MAVLink on UDP 14540+i.
- **Why:** by default every SIH instance spawns at the same spot. PX4 v1.17 reads per-instance home from the environment.
- **Files:** `launch_sih.py`, `geo.py`

### H-04: MAVSDK bridge
- **What:**
  1. take off;
  2. calibrate a shared ENU frame (GPS minus local position, averaged while hovering);
  3. switch to offboard;
  4. run PHASE at 5 Hz with position plus velocity-feedforward setpoints;
  5. land;
  6. write `flight.csv`, `meta.json` and `phase_ticks.json`.
- **Notes:** MAVSDK is pinned to `mavsdk==3.17.4`. Version 4.x replaced the asyncio API with a new native binding.
- **Files:** `phase_px4_bridge.py`, `requirements.txt`

### H-05: Analyzer
- **What:**
  - min separation of both **commanded** and **measured** positions;
  - violation ticks, completion, delay and path length;
  - lag-corrected tracking error;
  - PHASE compute time;
  - plots;
  - a kinematic baseline from the stock `Simulation.run()`.
- **Why commanded and measured are separate:** it tells algorithm violations apart from vehicle tracking lag.
- **Files:** `analyze.py`

### H-06: Tracking error compares the command with the *next* measurement
- **Why:** PHASE's target is deliberately one tick (0.8 m) ahead. Comparing it with the same-tick position reported that lead as about 0.78 m of "error".
- **Files:** `analyze.py`

### H-07: The bridge always exits, has a watchdog, and logs unbuffered
- **Why:** run `20261109_174223` stalled for about 1.5 h. Q3's simulated magnetometer timed out after takeoff, PX4 failsafed and landed, and the bridge raised an error but hung on shutdown because MAVSDK threads kept the process alive. Its log was empty because the pipe buffered the output.
- **What:**
  - `os._exit` on every path;
  - a 600 s watchdog;
  - exit code 3 for failures before PHASE takes control;
  - `run_all.sh` retries only exit 3;
  - `python -u` and line-buffered grep.
- **Files:** `phase_px4_bridge.py`, `run_all.sh`

### H-08: Pace PHASE on PX4's simulation clock, not the wall clock
- **Why:** measured sim/wall rate is **0.83×** on this Mac (`simrate` check, all 3 instances). With wall-clock pacing, each 0.2 s PHASE tick was only 0.166 s of flight, so drones looked 17% slow and PHASE's `dt` no longer matched the physics.
- **What:** the tick deadline uses odometry `time_usec`. `sim_s` and `sim_over_wall` are logged.
- **Files:** `phase_px4_bridge.py`
- **Known residual:** cruise speed is still 3.1–3.6 m/s against 4 m/s commanded. Not root-caused.

### H-09: Obstacle clearance metric
- **Files:** `analyze.py`

### F-01: V1.2 violates separation on real dynamics
- **Evidence:**
  - every V1.2 PX4 run had measured separation below 5.0 m: 1.79–4.21 m, 11–24 violation ticks;
  - commanded separation always stayed at 5.20 m.
- **Root cause:** conflicts are detected one tick (0.2 s) ahead at the bare margin, and responses assume an instant velocity change. Head-on, the drones closed at about 7 m/s, needed about 1.3 s to brake, and overshot by 3–4 m.

### F-02: V1.2 kinematic livelock in symmetric 3-way swap
- **Evidence:**
  - Q2 and Q3 never arrive in `Simulation.run()`; V1.2's own `verify_manifest()` reports FAILED;
  - Q2 cycles between three points near (−3.3, 9) for the full 300 s, logging 1,466 deflections;
  - the positional-deadlock detector never fires, because the vehicle moves about 0.85 m per tick.

### F-03: V1.2's deflection lasts one tick
- **What happens:** any new conflict triggers the global A* replan in the same tick, which resets every vehicle to NORMAL. The 90° evasion therefore never plays out.

---

## 2026-11-10: PHASE-D (dynamics-aware variant)

The user's direction: growing balls, priority braking, deflections that finish,
and subsequent deflections that follow the same rule. Design: `PHASE_D.md`.

### D-01: First PHASE-D
- **What:**
  - growing balls (margin + reaction + braking distance along the approach);
  - priority braking (lower momentum yields first; obstacles never yield);
  - acceleration-bounded commands;
  - deflections that run to completion;
  - re-deflection by the same rule;
  - one deflection per vehicle per tick, against the nearest new partner. Without that, the second of two same-tick deflections silently overwrote the first.
- **Files:** `phase_dynamic.py`, `test_phase_dynamic.py`, bridge `--variant phase_d`, `run_all.sh` `VARIANT=`, `compare.py`
- **Result:**
  - safe in kinematic runs: 6.5–16 m separation;
  - **worse on PX4** (runs `20261109_235853_phase_d`, `20261110_001035_phase_d`): separation down to 0.30 m, a drone 0.55 m inside the obstacle, and two scenarios timed out. See D-02 and D-03.

### D-02: Ramp commands from the commanded velocity, not the measured one
- **Why:** commands were limited to measured velocity ± 0.6 m/s per tick. PX4 turns that small velocity error into only about 1 m/s² of braking (velocity gain 1.8). Measured: commanded closing dropped about 1.4 m/s per second, while the balls assumed about 5 m/s² combined.
- **What:**
  - the internal commanded-velocity state is the ramp base;
  - measured velocity only sizes the balls (largest of measured, commanded and desired);
  - braking gets first call on the acceleration budget.

### D-03: Brake the velocity actually commanded this tick
- **Why:** braking checked the *desired* velocity, which after a deflection already points away, while the vehicle is still mostly on its old heading. Evidence: the 3-way case closed from 11 m to 4.2 m with zero brake events.
- **What:**
  - braking is applied to the post-acceleration-limit candidate velocity;
  - a speed cap is added (braking plus sidestepping had reached 4.21 m/s against the 4 m/s limit, caught by a test).

### D-04: `lag_plant.py`, an offline PX4-like tracking model
- **What:**
  - position P 0.95, velocity P 1.8, acceleration cap 5 m/s², 50 Hz;
  - reproduces V1.2's PX4 separations to within about 0.1 m (head-on 2.20 vs 2.21 m).
- **Why:** catches D-02/D-03-type bugs in seconds, without PX4.
- **Files:** `lag_plant.py`, test `test_separation_holds_with_px4_like_tracking_lag`

### D-05: Reaction time 0.4 s → 0.8 s; flight time limit 4× → 6× ideal + 30 s
- **Why:**
  - 0.4 s left 4.81 m under the lag model; a sweep gave 0.4 / 0.6 / 0.8 s → min separation 4.81 / 5.97 / 5.19 m and obstacle clearance 2.39 / 2.39 / 2.58 m;
  - 0.8 s ≈ tick (0.2 s) + PX4 velocity loop (0.56 s);
  - PHASE-D is slower and needed more than 80 s for the 3-way swap.
- **Result on PX4** (runs `20261110_002929_phase_d`, `20261110_005046_phase_d`):
  - 5 of 6 runs had zero violations;
  - the worst was head-on at 4.36 m (6 ticks);
  - obstacle clearance was 2.49 / 2.67 m;
  - delays roughly 2× V1.2's.

### D-06: Deflection direction scored against every nearby vehicle (user request 1a)
- **Why:** in the 4.36 m case, Q2's deflection away from Q1 pointed it towards Q3. PHASE's `calculate_90_deg_deflection` only knows about one partner. Commanded closing with Q3 rose from 1.1 to 2.3 m/s straight after.
- **What:**
  - same structure as PHASE's rule: 90° left or right of the heading, scored 3 × evasion + 1 × goal, with the same obstacle term;
  - the evasion vector sums a push away from **every vehicle inside the planning frame** (`astar_frame_size`, the same radius PHASE uses for localized A*);
  - each push is weighted by the distance at which the two balls would touch (`reach`).
- **Verified:**
  - with only the partner present and no obstacles, the choice is identical to PHASE's (50 random cases);
  - a third vehicle on the chosen side flips the choice;
  - vehicles outside the frame are ignored.
- **Files:** `phase_dynamic.py` (`_deflection_direction`), `test_spatial_and_response.py`

### D-07: Response-time parameter `tau_response` (user request 1b)
- **Why:** PX4's lag behaves like a first-order response, not a fixed delay. In the 4.36 m case the command was already moving apart at t = 8.6 s, but the real drones kept closing until t = 9.4 s.
- **What:**
  - `T_REACT` is split into `t_latency` (0.25 s: tick plus telemetry) and `tau_response` (0.556 s = 1/`MPC_XY_VEL_P_ACC`);
  - balls grow by c · (t_latency + tau);
  - braking subtracts the lag distance (v_meas − v_cmd)⁺ · tau from the gap: the extra distance a first-order system covers while it catches up with the command.
  - obstacles get the same treatment;
  - bridge flags `--tau-response` and `--t-latency`, recorded in `meta.json`.
- **Verified:**
  - a longer tau makes balls touch earlier;
  - a vehicle whose measured velocity is ahead of its command brakes harder;
  - under the lag model: separation 8.37 / 8.45 / 9.76 m and obstacle clearance 2.77 m. This is the first time clearance met 2.7 m.
  - **cost:** head-on completion time 41 → 100 s.

### D-08: Spatial hash grid for every neighbour and pair query (user question on O(N²))
- **Why:** conflict detection, braking, PBD, A* dynamic walls and the new D-06 scoring were all O(N²).
- **What:**
  - uniform grid with cell size = the largest distance at which two vehicles can still interact this tick (touching balls, or braking at closing speed 2·vmax including the lag term);
  - vehicles are hashed per tick in O(N), and queries look only at adjacent cells, giving O(N·k);
  - `use_grid=False` keeps the O(N²) reference; bridge flag `--no-grid`.
- **Verified:**
  - grid pairs and neighbours equal brute force on random point sets (15 cases);
  - PHASE-D trajectories with the grid are identical to brute force on all scenarios plus a random 30-vehicle fleet;
  - scaling: see `bench_scaling.py` and `runs/bench_scaling.txt`.
- **Files:** `spatial_grid.py`, `phase_dynamic.py`, `scenarios.build_random`, `bench_scaling.py`, `test_spatial_and_response.py`

### D-09: Replan only near a new conflict
- **Why:** the D-08 benchmark showed the grid fixed pair finding but ticks were still dominated by A*: about 0.57·N searches per tick (567 per tick at 1,000 vehicles). V1.2's rule replans every uninvolved vehicle on *any* new conflict, and in a big fleet there is always one somewhere.
- **What:** only NORMAL vehicles within the planning frame (`astar_frame_size`) of a vehicle in a new conflict replan. The others keep their paths.
- **Verified:**
  - 47 tests pass;
  - lag-model safety unchanged: 8.37 / 8.45 / 9.39 m, clearance 2.76 m;
  - tick time at 1,000 vehicles 1,557 → 199 ms, at 100 vehicles 53 → 10.5 ms;
  - head-on completion time 100 → 73 s.
- **Files:** `phase_dynamic.py`

### D-10: PX4 results for D-06 to D-09
- **Runs:** `20261110_152802_phase_d`, `20261110_154144_phase_d`.
- **Results:**
  - **zero separation violations in all 6 flights**, worst 7.11–9.07 m;
  - obstacle clearance 2.57 / 2.36 m (still below 2.7 m);
  - mean delays 35–64 s;
  - one 3-way flight timed out with Q2 18.5 m from its goal (see D-11).
- **Also fixed:** `compare.py` now takes each variant's kinematic row from its newest run directory. Before, it came from the oldest, i.e. stale code.

### D-11: Waypoint orbit fix
- **Why:** in the timed-out flight, Q2 was in NORMAL path-following with no conflicts, but its commanded velocity rotated steadily around the A* grid point (−12, 21) for about 20 s. An acceleration-limited vehicle can't turn tighter than r = v²/a_cmd (~3 m at 3 m/s). A waypoint inside that circle never comes within the 0.6 m pop radius, so the vehicle orbits it. The waypoint-following code dates from D-01.
- **What:** a waypoint is also dropped once it has been **passed**, meaning the vehicle is beyond it on the way to the next waypoint (dot(pos − wp, next − wp) > 0). Counters: `waypoints_reached` and `waypoints_passed`.
- **Verified:**
  - new regression test `test_no_orbiting_a_waypoint_inside_the_turning_circle`, which **fails on the old logic** (15.3 m of progress in 8 s instead of > 20 m) and passes now;
  - 48 tests pass;
  - lag model: completion times 72 / 73 / 88 s → **35 / 51 / 38 s**, separation 9.12 / 7.00 / 9.76 m, obstacle clearance 3.03 m;
  - waypoints were "passed" rather than reached 8–17 times per scenario, so orbiting and overshoot had been inflating PHASE-D's delays all along.
- **Files:** `phase_dynamic.py`, `test_phase_dynamic.py`

### H-10: 2-minute limit on bridge setup
- **Why:** in run `20261110_160830_phase_d` (head_on_cross), all three PX4 instances reported "Ready for takeoff", but a MAVSDK connect/health call never returned. Only the 600 s watchdog would end it.
- **What:** connect and health-wait are each bounded to 60 s. A timeout raises before PHASE takes control (exit 3), so `run_all.sh` retries.
- **Files:** `phase_px4_bridge.py`

### H-11: Start mavsdk_server processes up front (fixes ~60 s connect, and H-10's failures)
- **Why:**
  - after H-10, **all 12 flight attempts** failed at setup;
  - the setup limit wasn't the bug: connect had already slowed to about 2 min 20 s just before it;
  - step timing showed MAVSDK's `connect()` blocked the event loop for **62.7 s**;
  - `mavsdk_server` alone opens in 1.3 s, PX4 sends MAVLink within 1.5 s, and gRPC connects instantly to both `localhost` and `127.0.0.1`, which ruled out the server, PX4 and DNS;
  - **cause:** `System.connect()` forks `mavsdk_server` from a process where gRPC is already running for the previous drone ("Other threads are currently calling into gRPC, skipping fork() handlers").
- **What:**
  - the bridge spawns all `mavsdk_server`s **before any gRPC channel exists** and attaches with `System(mavsdk_server_address="127.0.0.1", port=…)`;
  - server logs go to `mavsdk_server_<i>.log` in the run directory;
  - servers are killed explicitly before `os._exit`.
- **Verified:**
  - standalone test: all three connected in 1.2 s and healthy in 4.4 s, against 62–140 s before;
  - H-10's 60 s limits are kept as a safety net.
- **Affected runs:** `20261110_160830_phase_d` (obstacle_choke never flew) and `20261110_162109_phase_d` (nothing flew) are setup failures, not PHASE results. The two flights that did complete in `20261110_160830_phase_d` (three_way_swap 44.8 s, head_on_cross 64.4 s) are valid D-11 results.
- **Files:** `phase_px4_bridge.py`

### D-12: PX4 results for D-11 (current version)
- **Runs:** `20261110_164322_phase_d`, `20261110_164920_phase_d`, with the H-11 bridge.
- **Results:**
  - **all 6 flights complete with 0 separation violations**, worst separation 7.20–8.92 m;
  - **obstacle clearance 2.88 / 2.84 m**, the first time the 2.7 m target is met on PX4;
  - **mean delays 18–29 s**, now comparable to V1.2's 15–40 s (D-10 had 35–64 s);
  - `pbd_unrealizable_ticks` = 0;
  - 14–24 re-deflections per flight, about half ending by the stall timer (see PHASE_D.md §8).
- **One startup retry:** PX4 put all three drones in HOLD before offboard started. Exit 3, retried automatically. Not a PHASE result.
- **Details:** `PHASE_D.md` §7, `runs/comparison_final.md`.
