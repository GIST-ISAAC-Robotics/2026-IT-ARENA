#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_dense_stop_v1
test ! -e "$base/initial_s0"
test ! -e "$base/initial_s0.runner.log"
set +e
python3 -B scripts/validate_g1_traffic.py --case initial_occlusion --seed 0 --mode g1 --config config/tests/g1_traffic_extensions_v1.json --output "$base/initial_s0" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate > "$base/initial_s0.runner.log" 2>&1
run_exit=$?
set -e
printf '%s\n' "$run_exit" > "$base/validator_exit.txt"
tail -25 "$base/initial_s0.runner.log"
if test "$run_exit" -ne 0; then
  exit "$run_exit"
fi
python3 -B scripts/replay_follow_trace.py "$base/initial_s0/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
python3 -B scripts/audit_g1_traffic_coverage.py "$base/initial_s0" --output "$base/postcoverage.json" > "$base/postcoverage.log" 2>&1
tail -8 "$base/replay.log"
tail -8 "$base/postcoverage.log"
