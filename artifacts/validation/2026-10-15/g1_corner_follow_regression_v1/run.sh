#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corner_follow_regression_v1
cfg=config/tests/g1_traffic_extensions_v1.json
test ! -e "$base/runner.log"
if (( $(date -u +%s) >= 1792035000 )); then echo wrap_up_deadline_reached; exit 0; fi
set +e
python3 -B scripts/validate_g1_traffic.py --case first_curve --seed 0 --mode g1 --config "$cfg" --output "$base/run" --record-braking-diagnostics --record-follow-decisions --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate > "$base/runner.log" 2>&1
validator_exit=$?
python3 -B scripts/replay_follow_trace.py "$base/run/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
replay_exit=$?
python3 -B scripts/audit_g1_case_evidence.py "$base/run" --config "$cfg" --output "$base/case_evidence.json" > "$base/case_evidence.log" 2>&1
case_exit=$?
python3 -B scripts/audit_follow_occupancy.py "$base/run/follow_decisions.jsonl" --mode per_ray_candidate --output "$base/occupancy.json" > "$base/occupancy.log" 2>&1
occupancy_exit=$?
python3 -B scripts/summarize_corner_trace.py "$base/run/follow_decisions.jsonl" --output "$base/corner_summary.json" > "$base/corner_summary.log" 2>&1
summary_exit=$?
set -e
printf 'validator=%s\nreplay=%s\ncase=%s\noccupancy=%s\nsummary=%s\n' "$validator_exit" "$replay_exit" "$case_exit" "$occupancy_exit" "$summary_exit" > "$base/exit.txt"
if (( validator_exit != 0 || replay_exit != 0 || case_exit != 0 || occupancy_exit != 0 || summary_exit != 0 )); then exit 1; fi
