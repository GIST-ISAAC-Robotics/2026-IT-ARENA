#!/usr/bin/env bash
# run21: 사전 등록(RIGHT_ENCOUNTER_DRIVE_PLAN.md)한 오른쪽 옆차 command_fallback ON 1회.
# 주행 1회 -> 판단 재생 1회 -> 노면 평가 1회. 각 단계는 결과 파일이 이미 있으면 다시 실행하지 않는다.
set -uo pipefail
ROOT="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
D="artifacts/validation/2026-10-25/m3_execution_review_v1/run21_right_encounter"
cd "$ROOT" || exit 90

if [ -e "$D/run" ] || [ -e "$D/exit_code.txt" ]; then
  echo "run21 output already exists; refusing duplicate run" | tee -a "$D/duplicate_refused.txt"
  exit 91
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1

hash_files() { while IFS= read -r f; do sha256sum "$f"; done < "$D/files_to_hash.txt"; }

{ echo "ROS_DISTRO=${ROS_DISTRO:-unset}"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}";
  echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset}"; echo "PYTHONPATH=${PYTHONPATH:-}";
  echo "started_local=$(date --iso-8601=seconds)"; } > "$D/environment.txt"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2|parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$D/preexisting_processes.txt" || true
if [ -s "$D/preexisting_processes.txt" ]; then echo "preexisting simulation processes; not starting" | tee "$D/ABORTED_PREEXISTING.txt"; exit 92; fi
hash_files > "$D/source_sha256_before.txt"
python3 -B "artifacts/validation/2026-10-25/m3_execution_review_v1/run12_selection_integration/install_import_check.py" "$D/install_import_results.json" > "$D/install_import_stdout.txt" 2> "$D/install_import_stderr.txt"
echo $? > "$D/install_import_exit_code.txt"
python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); assert all(x["match"] for x in r.values())' "$D/install_import_results.json" || exit 93

CMD=(python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/e3_m3_right_grid_v1.json --actor-overlay config/tests/e3_onroad_right_encounter_v1.json --output "$D/run")
printf '%q ' "${CMD[@]}" > "$D/command.txt"; echo >> "$D/command.txt"
echo "drive_start_local=$(date --iso-8601=seconds)" >> "$D/environment.txt"
"${CMD[@]}" > "$D/stdout.txt" 2> "$D/stderr.txt"
echo $? > "$D/exit_code.txt"
echo "drive_end_local=$(date --iso-8601=seconds)" >> "$D/environment.txt"
hash_files > "$D/source_sha256_after.txt"
diff "$D/source_sha256_before.txt" "$D/source_sha256_after.txt" > "$D/hash_diff.txt"; echo "diff_exit=$?" >> "$D/hash_diff.txt"
sleep 3
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2|parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$D/residual_processes.txt" || true

if [ -f "$D/run/follow_decisions.jsonl" ] && [ ! -e "$D/follow_trace_replay.json" ]; then
  RCMD=(python3 -B scripts/replay_follow_trace.py "$D/run/follow_decisions.jsonl" --output "$D/follow_trace_replay.json")
  printf '%q ' "${RCMD[@]}" > "$D/replay_command.txt"; echo >> "$D/replay_command.txt"
  "${RCMD[@]}" > "$D/replay_stdout.txt" 2> "$D/replay_stderr.txt"
  echo $? > "$D/replay_exit_code.txt"
fi
if [ -d "$D/run" ] && [ ! -e "$D/surface_contact_eval.json" ]; then
  SCMD=(python3 -B scripts/surface_contact_eval.py "$D/run" --output "$D/surface_contact_eval.json")
  printf '%q ' "${SCMD[@]}" > "$D/surface_eval_command.txt"; echo >> "$D/surface_eval_command.txt"
  "${SCMD[@]}" > "$D/surface_eval_stdout.txt" 2> "$D/surface_eval_stderr.txt"
  echo $? > "$D/surface_eval_exit_code.txt"
fi
hash_files > "$D/source_sha256_after_postprocess.txt"
diff "$D/source_sha256_before.txt" "$D/source_sha256_after_postprocess.txt" > "$D/hash_diff_after_postprocess.txt"; echo "diff_exit=$?" >> "$D/hash_diff_after_postprocess.txt"
ps -eo pid,etime,cmd | grep -E 'gz sim|gzserver|ros2|parameter_bridge|local_pursuit|lidar_safety|validate_g1|ruby' | grep -v grep > "$D/residual_processes_after_postprocess.txt" || true
echo "finished_local=$(date --iso-8601=seconds)" >> "$D/environment.txt"
echo "RUN21_DONE drive_exit=$(cat "$D/exit_code.txt") replay_exit=$(cat "$D/replay_exit_code.txt" 2>/dev/null || echo none) surface_exit=$(cat "$D/surface_eval_exit_code.txt" 2>/dev/null || echo none)" | tee "$D/DONE.txt"
