#!/usr/bin/env bash
# 10/21 관측 계산 변경과 최종 진단 필드의 ROS 연결 검사 1회. Gazebo/실물 시험 아님.
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-21/g1_observation_ros_v1"
RUN="$ARTIFACT/run"
cd "$ROOT"
if [[ -e "$RUN" ]]; then printf 'Fresh evidence path required\n' >&2; exit 2; fi
mkdir -p "$RUN"
printf '%s\n' 'python3 -B scripts/validate_pipeline_faults_ros.py --controller-profile g1_recent_observation_candidate --scenario full --wall-cap 600 --recent-check-mode evidence_expiry --output artifacts/validation/2026-10-21/g1_observation_ros_v1/run/validator_output' > "$RUN/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_before.sha256"
tar -czf "$RUN/source_snapshot_before.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$RUN/source_hashes_after.sha256"
  if diff -u "$RUN/source_hashes_before.sha256" "$RUN/source_hashes_after.sha256" > "$RUN/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$RUN/source_hash_status.txt"; else echo SOURCE_CHANGED > "$RUN/source_hash_status.txt"; fi
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
  python3 -c 'import ackermann_msgs; import arena_autonomy.observation_support as o; print("ackermann_msgs="+ackermann_msgs.__file__); print("observation_support="+o.__file__)'
  nproc; uname -r; cat /proc/loadavg
  ls /proc/pressure 2>&1 || true
  echo 'host_power_note=No power setting changed for this run; current mode not remeasured'
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
