#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_right_candidate_v1
test ! -e "$base/side_right_s0"
test ! -e "$base/side_right_s0.runner.log"
set +e
python3 -B scripts/validate_g1_traffic.py --case side_right --seed 0 --mode g1 --config config/tests/g1_side_occlusion_v1.json --output "$base/side_right_s0" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate > "$base/side_right_s0.runner.log" 2>&1
run_exit=$?
set -e
echo "G1 validator exit=$run_exit"
tail -35 "$base/side_right_s0.runner.log"
if test -f "$base/side_right_s0/follow_decisions.jsonl"; then
  python3 -B scripts/replay_follow_trace.py "$base/side_right_s0/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
  tail -10 "$base/replay.log"
fi
if test -f "$base/side_right_s0/report.json"; then
  python3 -B scripts/audit_g1_traffic_coverage.py "$base/side_right_s0" --output "$base/postcoverage.json" > "$base/postcoverage.log" 2>&1
  tail -5 "$base/postcoverage.log"
fi
exit "$run_exit"
