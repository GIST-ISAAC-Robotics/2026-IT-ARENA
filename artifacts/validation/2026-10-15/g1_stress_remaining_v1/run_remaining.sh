#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_stress_remaining_v1
cfg=config/tests/g1_traffic_extensions_v1.json
for item in initial_occlusion:bias first_curve:delay; do
  if (( $(date -u +%s) >= 1792002600 )); then
    echo 'wrap_up_deadline_reached_before_new_case'
    exit 0
  fi
  case_name="${item%%:*}"
  condition="${item##*:}"
  if [[ "$condition" == bias ]]; then
    profile=config/tests/lidar_positive_bias30.json
  else
    profile=config/tests/lidar_uniform30_delay50_jitter20.json
  fi
  name="${case_name}_${condition}_s0"
  test ! -e "$base/$name"
  test ! -e "$base/$name.runner.log"
  set +e
  python3 -B scripts/validate_g1_traffic.py --case "$case_name" --seed 0 --mode g1 --config "$cfg" --output "$base/$name" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate --occupancy-timing per_ray_candidate --lidar-test-profile "$profile" > "$base/$name.runner.log" 2>&1
  run_exit=$?
  replay_exit=1
  audit_exit=1
  if test -f "$base/$name/follow_decisions.jsonl"; then
    python3 -B scripts/replay_follow_trace.py "$base/$name/follow_decisions.jsonl" --output "$base/$name.replay.json" > "$base/$name.replay.log" 2>&1
    replay_exit=$?
  fi
  if test -f "$base/$name/report.json"; then
    python3 -B scripts/audit_g1_case_evidence.py "$base/$name" --config "$cfg" --output "$base/$name.case_evidence.json" > "$base/$name.case_evidence.log" 2>&1
    audit_exit=$?
  fi
  set -e
  printf 'validator=%s\nreplay=%s\ncase_evidence=%s\n' "$run_exit" "$replay_exit" "$audit_exit" > "$base/$name.exit.txt"
  echo "case=$name validator_exit=$run_exit replay_exit=$replay_exit audit_exit=$audit_exit"
  tail -15 "$base/$name.runner.log"
  if (( run_exit != 0 || replay_exit != 0 || audit_exit != 0 )); then exit 1; fi
done
