#!/usr/bin/env bash
# v8 통과 뒤 같은 2번째 재시작의 임계 미만 정지 지점을 판별하는 진단 1회. 10/17 v6와 같은 시나리오·프로파일·한도. 추가된 것은 시험 전용 명령 경로 진단과
# 재시작 사례의 MCU ARMED 유지 기대, 느린 발행·spin·스레드 표본·/dev/shm 목록뿐이다. 실패해도 자동 반복하지 않는다.
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-19/restart_command_path_v1/ros_v9"
RUN="$ARTIFACT/run"
KEYS='src/arena_autonomy/arena_autonomy/observation_support.py scripts/validate_pipeline_faults_ros.py scripts/pipeline_mcu_rejection_probe.py scripts/pipeline_restart_probe.py src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py src/arena_autonomy/arena_autonomy/lidar_safety.py'
cd "$ROOT"
mkdir -p "$RUN"
printf '%s\n' \
  'python3 -B scripts/validate_pipeline_faults_ros.py --controller-profile g1_recent_observation_candidate --scenario full --wall-cap 600 --recent-check-mode evidence_expiry --output artifacts/validation/2026-10-19/restart_command_path_v1/ros_v9/run/validator_output' > "$RUN/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_before.sha256"
sha256sum $KEYS > "$RUN/runtime_key_hashes_before.sha256"
tar -czf "$RUN/source_snapshot_before.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_after.sha256"
  if diff -u "$RUN/source_hashes_before.sha256" "$RUN/source_hashes_after.sha256" > "$RUN/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$RUN/source_hash_status.txt"; else echo SOURCE_CHANGED > "$RUN/source_hash_status.txt"; fi
  sha256sum $KEYS > "$RUN/runtime_key_hashes_after.sha256"
  diff -u "$RUN/runtime_key_hashes_before.sha256" "$RUN/runtime_key_hashes_after.sha256" > "$RUN/runtime_key_hash_diff.txt" || true
  ps -eo pid,ppid,pgid,sid,stat,etimes,args > "$RUN/process_table_after.txt" 2>&1 || true
  echo "EXIT_STATUS=$result" > "$RUN/runner_status.txt"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
{
  date --iso-8601=seconds
  printf 'ROS_DISTRO=%s\nRMW_IMPLEMENTATION=%s\nPYTHONPATH=%s\n' "${ROS_DISTRO-}" "${RMW_IMPLEMENTATION-}" "$PYTHONPATH"
  python3 -c 'import ackermann_msgs; print("ackermann_msgs="+ackermann_msgs.__file__)'
  nproc; uname -r; cat /proc/loadavg
  ls /proc/pressure 2>&1 || true
  echo 'host_power_note=Windows power plan Turbo and Armoury Crate Turbo set by main agent before this run (reported 12:31 KST); recorded as environment only'
} > "$RUN/environment.log" 2>&1
ps -eo pid,ppid,pgid,sid,stat,etimes,args > "$RUN/process_table_before.txt" 2>&1 || true
set +e
python3 -B scripts/validate_pipeline_faults_ros.py \
  --controller-profile g1_recent_observation_candidate \
  --scenario full --wall-cap 600 --recent-check-mode evidence_expiry \
  --output "$RUN/validator_output" > "$RUN/validator.log" 2>&1
status=$?
set -e
exit "$status"
