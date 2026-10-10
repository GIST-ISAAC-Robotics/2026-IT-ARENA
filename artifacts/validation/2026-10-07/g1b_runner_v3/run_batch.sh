#!/usr/bin/env bash
# G1-b 재개 행렬을 순차 실행한다. 인자: case:seed:mode:출력이름 ...
# 각 실행 전 시험 잔존 프로세스가 있으면 즉시 중단한다(강제 종료하지 않음).
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
runner='artifacts/validation/2026-10-07/g1b_runner_v3/run_case.sh'
for item in "$@"; do
  IFS=: read -r case_name seed mode name <<<"$item"
  out="artifacts/validation/2026-10-07/$name"
  if [ -e "$out" ]; then echo "SKIP_EXISTS $name"; exit 94; fi
  residual=$(ps -eo pid,cmd | grep -E 'gz sim|validate_g1_traffic|parameter_bridge|rotating_lidar|actuation_ros|local_pursuit|lidar_safety' | grep -v grep)
  if [ -n "$residual" ]; then echo "RESIDUAL_BEFORE $name"; echo "$residual"; exit 95; fi
  started=$(date -Iseconds)
  bash "$runner" "$case_name" "$seed" "$mode" "$out" > /dev/null 2>&1
  rc=$?
  summary=$(python3 -c "import json,sys;r=json.load(open('$out/report.json'));print(json.dumps({k:r.get(k) for k in ('passed','run_valid','harness_valid','behavior_gates_passed','shutdown_clean','source_unchanged','abort','wall_elapsed_s')}))" 2>/dev/null || echo '{"report":"missing"}')
  echo "RUN $name case=$case_name seed=$seed mode=$mode rc=$rc started=$started $summary"
done
echo "BATCH_DONE"
