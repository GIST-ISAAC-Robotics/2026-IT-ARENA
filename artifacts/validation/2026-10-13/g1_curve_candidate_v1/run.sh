#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_curve_candidate_v1
test ! -e "$base/first_curve_s0"
test ! -e "$base/first_curve_s0.runner.log"
set +e
python3 -B scripts/validate_g1_traffic.py --case first_curve --seed 0 --mode g1 --config config/tests/g1_traffic_extensions_v1.json --output "$base/first_curve_s0" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate > "$base/first_curve_s0.runner.log" 2>&1
run_exit=$?
set -e
echo "G1 validator exit=$run_exit"
tail -35 "$base/first_curve_s0.runner.log"
if test -f "$base/first_curve_s0/follow_decisions.jsonl"; then
  python3 -B scripts/replay_follow_trace.py "$base/first_curve_s0/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
  tail -10 "$base/replay.log"
fi
exit "$run_exit"
