#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_stress_stopped_v1
cfg=config/tests/g1_traffic_v1.json
name=stopped_center_bias_s0
test ! -e "$base/$name"
test ! -e "$base/runner.log"
if (( $(date -u +%s) >= 1792002600 )); then
  echo wrap_up_deadline_reached_before_execution
  exit 0
fi
set +e
python3 -B scripts/validate_g1_traffic.py --case stopped_center --seed 0 --mode g1 --config "$cfg" --output "$base/$name" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate --occupancy-timing per_ray_candidate --lidar-test-profile config/tests/lidar_positive_bias30.json > "$base/runner.log" 2>&1
run_exit=$?
python3 -B scripts/replay_follow_trace.py "$base/$name/follow_decisions.jsonl" --output "$base/replay.json" > "$base/replay.log" 2>&1
replay_exit=$?
python3 -B scripts/audit_g1_case_evidence.py "$base/$name" --config "$cfg" --output "$base/case_evidence.json" > "$base/case_evidence.log" 2>&1
audit_exit=$?
python3 -B scripts/audit_follow_occupancy.py "$base/$name/follow_decisions.jsonl" --mode per_ray_candidate --output "$base/occupancy.json" > "$base/occupancy.log" 2>&1
occupancy_exit=$?
set -e
printf 'validator=%s\nreplay=%s\ncase_evidence=%s\noccupancy=%s\n' "$run_exit" "$replay_exit" "$audit_exit" "$occupancy_exit" > "$base/exit.txt"
tail -10 "$base/runner.log"
if (( run_exit != 0 || replay_exit != 0 || audit_exit != 0 || occupancy_exit != 0 )); then exit 1; fi
