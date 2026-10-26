#!/usr/bin/env bash
# B3 선택형 경기 감독 제한 ROS 1회. Gazebo/실물 시험 아님.
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-24/b3_race_session_ros_v1"
cd "$ROOT"
if [[ -e "$ARTIFACT/runner_status.txt" || -e "$ARTIFACT/run" ]]; then printf 'Fresh evidence path required\n' >&2; exit 2; fi
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/source_hashes_before.sha256"
tar --exclude=__pycache__ -czf "$ARTIFACT/source_snapshot_before.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/source_hashes_after.sha256"
  if diff -u "$ARTIFACT/source_hashes_before.sha256" "$ARTIFACT/source_hashes_after.sha256" > "$ARTIFACT/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$ARTIFACT/source_hash_status.txt"; else echo SOURCE_CHANGED > "$ARTIFACT/source_hash_status.txt"; result=1; fi
  ps -eo pid,ppid,pgid,sid,stat,etimes,args > "$ARTIFACT/process_table_after.txt" 2>&1 || true
  echo "EXIT_STATUS=$result" > "$ARTIFACT/runner_status.txt"
  exit "$result"
}
trap finish EXIT
{
  date --iso-8601=seconds
  printf 'ROS_DISTRO=%s\nRMW_IMPLEMENTATION=%s\n' "${ROS_DISTRO-}" "${RMW_IMPLEMENTATION-}"
  python3 -c 'import arena_autonomy.race_session as r; print("race_session="+r.__file__)'
  nproc; uname -r; cat /proc/loadavg
  echo 'host_power_note=No power setting changed for this run'
} > "$ARTIFACT/environment.log" 2>&1
ps -eo pid,ppid,pgid,sid,stat,etimes,args > "$ARTIFACT/process_table_before.txt" 2>&1 || true
python3 -B -m pytest -q -p no:cacheprovider tests/test_race_session.py tests/test_race_session_node.py tests/test_race_permission.py tests/test_race_actuation_contract.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py tests/test_official_start_signal.py tests/test_official_vision_pipeline.py tests/test_vision_process.py tests/test_basic_autonomy.py tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py tests/test_r0_plus.py tests/test_r0_plus_integration.py tests/test_local_follow.py tests/test_follow_trace.py tests/test_lidar_safety.py tests/test_signal_mask.py --junitxml="$ARTIFACT/junit.xml" > "$ARTIFACT/tests.log" 2>&1
tail -3 "$ARTIFACT/tests.log"
set +e
python3 -B scripts/validate_pipeline_faults_ros.py --scenario race_session --signal-mode official_4red_candidate \
  --output "$ARTIFACT/run" --wall-cap 600 --sleep-wall .005 > "$ARTIFACT/validator.log" 2>&1
status=$?
set -e
tail -5 "$ARTIFACT/validator.log"
exit "$status"
