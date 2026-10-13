#!/usr/bin/env bash
# 평가기 연결과 최종 검토 뒤에만 실행한다. 각 조건 1회, 첫 실패에서 중단.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_lidar_stress_v1
for item in 'side_left:uniform' 'initial_occlusion:bias' 'first_curve:delay'; do
  case_name="${item%%:*}"
  condition="${item##*:}"
  case "$condition" in
    uniform) profile=config/tests/lidar_uniform30.json ;;
    bias) profile=config/tests/lidar_positive_bias30.json ;;
    delay) profile=config/tests/lidar_uniform30_delay50_jitter20.json ;;
  esac
  if [[ "$case_name" == side_left ]]; then
    cfg=config/tests/g1_side_occlusion_v1.json
  else
    cfg=config/tests/g1_traffic_extensions_v1.json
  fi
  name="${case_name}_${condition}_s0"
  test ! -e "$base/$name"
  test ! -e "$base/$name.runner.log"
  set +e
  python3 -B scripts/validate_g1_traffic.py --case "$case_name" --seed 0 --mode g1 --config "$cfg" --output "$base/$name" --record-braking-diagnostics --record-follow-decisions --boundary-evidence fragmented_candidate --lidar-test-profile "$profile" > "$base/$name.runner.log" 2>&1
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
  tail -20 "$base/$name.runner.log"
  if (( run_exit != 0 || replay_exit != 0 || audit_exit != 0 )); then exit 1; fi
done
