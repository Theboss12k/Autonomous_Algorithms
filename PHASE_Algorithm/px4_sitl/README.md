# PHASE on PX4 SITL (3 quadcopters, SIH)

Runs PHASE in closed loop with real PX4 flight stacks. Three headless PX4 SIH
quadcopters (`sihsim_quadx`, airframe 10040) fly while PHASE, ticking at its own
`dt = 0.2 s`, reads their **measured** positions and drives them with offboard
position + velocity-feedforward setpoints.

The PHASE source file is loaded unmodified. By default that's
`PHASE_V1.2/Testing/phase_altitude_locked.py` in the main `Autonomous_Algorithms`
checkout next to this worktree. Point `PHASE_SRC=/path/to/phase.py` at any other version.

## Files

| file | role |
|---|---|
| `phase_stepper.py` | Subclass of PHASE's `Simulation` whose `step(measured)` runs one iteration of `run()`'s loop, with `current_pos` taken from telemetry |
| `test_stepper_equivalence.py` | With perfect tracking, `step()` must reproduce `Simulation.run()` exactly (keeps the copy in sync with the algorithm) |
| `scenarios.py` | `three_way_swap`, `head_on_cross`, `obstacle_choke`: quad scale, 4 m/s, 2.5 m margins, 10 m altitude |
| `launch_sih.py` | Starts one PX4 SIH instance per vehicle, spawned at its start point (`PX4_HOME_LAT/LON`), offboard MAVLink on UDP 14540+i |
| `phase_px4_bridge.py` | MAVSDK: take off, calibrate a shared ENU frame, offboard, PHASE loop at 5 Hz, land; writes `flight.csv`, `meta.json`, `phase_ticks.json` |
| `analyze.py` | Runs the stock kinematic `Simulation.run()` as a baseline, and compares: min separation (commanded and measured), violations, delay, path length, lag-corrected tracking error, PHASE compute time. Writes `report.json`, `paths.html` and `summary.md` |
| `run_all.sh` | Fresh PX4 instances per scenario, fly, analyze (`VARIANT=v12` or `VARIANT=phase_d`) |
| `phase_dynamic.py` | **PHASE-D** experimental variant: growing balls (margin + reaction + braking distance along the approach), priority braking (lower momentum yields first, obstacles never yield), acceleration-bounded commands, deflections that run to completion, subsequent deflections by the same rule |
| `test_phase_dynamic.py` | Tests for each PHASE-D rule, kinematic safety, and closed-loop safety against `lag_plant` |
| `lag_plant.py` | Crude PX4-like tracking model (MPC_XY_P / MPC_XY_VEL_P_ACC / MPC_ACC_HOR_MAX) for fast offline closed-loop checks. It reproduced V1.2's PX4 separations to within ~0.1 m |
| `compare.py` | Side-by-side table of several run directories |
| `spatial_grid.py` | Uniform spatial hash grid: O(N·k) neighbour and pair queries instead of O(N²) |
| `test_spatial_and_response.py` | Grid = brute force; PHASE-D identical with and without the grid; all-nearby deflection scoring; response-time parameter |
| `bench_scaling.py` | Tick cost vs fleet size (10–1,000 vehicles), grid vs brute force |
| **`CHANGELOG.md`** | Every change, with ID, reason, evidence, files and verification |
| **`PHASE_D.md`** | PHASE-D design: rule-by-rule diff from V1.2, equations, parameters, results, limitations |

## Setup (macOS, Apple Silicon)

```bash
brew install cmake ninja ccache
git clone --branch v1.17.0 --recurse-submodules https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && python3 -m venv .venv && .venv/bin/pip install future -r Tools/setup/requirements.txt
PATH="$PWD/.venv/bin:$PATH" make px4_sitl_default

cd <this folder>
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

(The `PX4/px4/px4-dev` Homebrew formula is now an empty stub, so install the tools directly as above.)

## Run

```bash
.venv/bin/python -m pytest -q          # stepper equivalence
./run_all.sh                           # all scenarios -> runs/<timestamp>/summary.md
./run_all.sh head_on_cross             # one scenario
VARIANT=phase_d ./run_all.sh           # the PHASE-D variant
.venv/bin/python compare.py runs/<a> runs/<b> ...
```

Manual, two terminals:

```bash
.venv/bin/python launch_sih.py --scenario head_on_cross --out runs/manual
.venv/bin/python phase_px4_bridge.py --scenario head_on_cross --out runs/manual
```

QGroundControl, if you have it, connects automatically on UDP 14550 and shows all three vehicles.

## Notes

- Frames: PHASE x/y/z = East/North/Up. Each drone's local NED origin is located in a
  shared ENU frame (anchored at PX4's default SIH home) by averaging GPS minus local
  position while hovering.
- Separation is reported for both the positions PHASE **commanded** and the positions the
  drones **actually** reached. A violation in the measured column but not the commanded
  column comes from tracking lag (vehicle dynamics), not from PHASE's geometry.
- The commanded velocity is clamped to 1.5 × vehicle speed. Each clamp is counted
  (`vel_clamps`), since it marks a PBD jump the quad could not follow in one tick.
