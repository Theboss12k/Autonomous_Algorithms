#!/usr/bin/env bash
# Fly every scenario on fresh PX4 SIH instances, then analyze.
#   ./run_all.sh [scenario ...]      (default: all scenarios)
# A scenario is retried (up to RETRIES times) only when PX4 fails before PHASE
# takes control (bridge exit 3), e.g. an intermittent SIH sensor timeout.
set -uo pipefail
cd "$(dirname "$0")"
PY=".venv/bin/python -u"
RETRIES=${RETRIES:-2}
VARIANT=${VARIANT:-v12}
RUN="runs/$(date +%Y%m%d_%H%M%S)_$VARIANT"
mkdir -p "$RUN"
SCEN=("$@")
[ ${#SCEN[@]} -eq 0 ] && SCEN=($($PY -c "import scenarios; print(' '.join(scenarios.SCENARIOS))"))
FILTER="fork_posix|mavsdk-grpc|pin 'mavsdk|migration.html|from mavsdk|FutureWarning"

stop_px4() {
  [ -n "${LPID:-}" ] && kill -TERM "$LPID" 2>/dev/null && wait "$LPID" 2>/dev/null
  pkill -x px4 2>/dev/null; pkill -f mavsdk_server 2>/dev/null
  sleep 2
}
trap 'stop_px4; exit 130' INT TERM

for s in "${SCEN[@]}"; do
  for attempt in $(seq 0 "$RETRIES"); do
    echo "=== $s (attempt $((attempt + 1))) ==="
    rm -rf "$RUN/$s"; mkdir -p "$RUN/$s"
    $PY launch_sih.py --scenario "$s" --out "$RUN/$s" > "$RUN/$s/launcher.log" 2>&1 &
    LPID=$!
    sleep 3
    $PY phase_px4_bridge.py --scenario "$s" --variant "$VARIANT" --out "$RUN/$s" 2>&1 \
      | grep --line-buffered -v -E "$FILTER" | tee "$RUN/$s/bridge.log"
    rc=${PIPESTATUS[0]}
    stop_px4
    [ "$rc" -ne 3 ] && break
    echo "--- $s: PX4 failed before PHASE started; retrying"
    mkdir -p "$RUN/_failed_attempts" && cp -r "$RUN/$s" "$RUN/_failed_attempts/${s}_$attempt"
  done
  echo "$s exit $rc" >> "$RUN/exit_codes.txt"
done

$PY analyze.py "$RUN"
echo "results: $RUN/summary.md"
