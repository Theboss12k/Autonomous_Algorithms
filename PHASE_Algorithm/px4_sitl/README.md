# PHASE on PX4 SITL (3 quadcopters, SIH)

Runs PHASE in closed loop with real PX4 flight stacks. Three headless PX4 SIH quadcopters (`sihsim_quadx`,
airframe 10040) fly while PHASE reads their **measured** positions and drives them with offboard
position + velocity setpoints. PHASE ticks at its own `dt = 0.2 s`, paced on PX4's simulation clock.

`--variant v12` flies PHASE V1.2 itself, from `../versions/V1.2/`, unmodified. `--variant phase_d` flies PHASE-D from `../phase_d/`.
`PHASE_SRC=V1.1` (or a file path) swaps the base version.

## Files

| File | Role |
|---|---|
| `launch_sih.py` | Starts one PX4 SIH instance per vehicle, spawned at its start point (`PX4_HOME_LAT/LON`), offboard MAVLink on UDP 14540+i |
| `phase_px4_bridge.py` | MAVSDK: take off, calibrate a shared ENU frame, offboard, PHASE loop at 5 Hz, land. Writes `flight.csv`, `meta.json`, `phase_ticks.json` |
| `analyze.py` | Runs the same variant kinematically as a baseline and compares: min separation (commanded and measured), violations, obstacle clearance, delay, lag-corrected tracking error. Writes `report.json`, `paths.html` and `summary.md` |
| `compare.py` | Side-by-side table of several run directories |
| `run_all.sh` | Fresh PX4 instances per scenario, fly, analyze (`VARIANT=v12` or `VARIANT=phase_d`) |
| `geo.py` | Conversion between the shared ENU frame and latitude/longitude |

Scenarios (`three_way_swap`, `head_on_cross`, `obstacle_choke`), the tick-by-tick stepper and PHASE-D itself live
in [`../phase_d/`](../phase_d). Every change and result is logged in [`../CHANGELOG.md`](../CHANGELOG.md).

## Setup (macOS, Apple Silicon)

```bash
brew install cmake ninja ccache
git clone --branch v1.17.0 --recurse-submodules https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && python3 -m venv .venv && .venv/bin/pip install future -r Tools/setup/requirements.txt
PATH="$PWD/.venv/bin:$PATH" make px4_sitl_default

cd <repo>/PHASE_Algorithm
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

(The `PX4/px4/px4-dev` Homebrew formula is now an empty stub, so install the tools directly as above.)
`launch_sih.py` expects PX4 at `~/PX4-Autopilot`; set `PX4_DIR` if it is elsewhere.

## Run

From `PHASE_Algorithm/px4_sitl/`:

```bash
./run_all.sh                           # all scenarios, PHASE V1.2 -> runs/<timestamp>_v12/summary.md
./run_all.sh head_on_cross             # one scenario
VARIANT=phase_d ./run_all.sh           # PHASE-D
../.venv/bin/python compare.py runs/<a> runs/<b> ...
```

`run_all.sh` uses `$PYTHON` if set, otherwise `../.venv`. Manually, in two terminals:

```bash
../.venv/bin/python launch_sih.py --scenario head_on_cross --out runs/manual
../.venv/bin/python phase_px4_bridge.py --scenario head_on_cross --out runs/manual
```

QGroundControl, if you have it, connects automatically on UDP 14550 and shows all three vehicles.

## Notes

- **Frames:** PHASE x/y/z are East/North/Up. Each drone's local NED origin is placed in a shared ENU frame
  (anchored at PX4's default SIH home) by averaging GPS minus local position while hovering.
- **Separation** is reported for both the positions PHASE **commanded** and the positions the drones
  **actually** reached. A violation in the measured column but not the commanded one comes from tracking lag
  (vehicle dynamics), not from PHASE's geometry.
- **Velocity clamp:** the commanded velocity is clamped to 1.5 × vehicle speed. Each clamp is counted
  (`vel_clamps`), since it marks a PBD jump the quad could not follow in one tick.
- **Flight data:** `runs/` is git-ignored. Raw flight data stays on the machine that flew it.
