#!/usr/bin/env bash
# 검토 정정(호스트 진단 호출 전 값 복사·시작 그래프 계약 회귀) 뒤 관련 회귀 1회. 전체 3,041회와 별개.
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ART="$ROOT/artifacts/validation/2026-10-19/restart_command_path_v1/regression_related_after_review"
cd "$ROOT"
TESTS='tests/test_pipeline_restart_probe.py tests/test_pipeline_restart_trace.py tests/test_pipeline_mcu_rejection_probe.py tests/test_pipeline_fault_validation.py tests/test_recent_observation_pipeline.py tests/test_official_start_ros_fixture.py tests/test_actuation_contract.py tests/test_motor_feedback_fault_classification.py'
printf '%s\n' "python3 -B -m pytest -q -p no:cacheprovider $TESTS" > "$ART/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B -m pytest -q -p no:cacheprovider $TESTS > "$ART/pytest.log" 2>&1
status=$?
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_after.sha256"
if diff -u "$ART/source_hashes_before.sha256" "$ART/source_hashes_after.sha256" > "$ART/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$ART/source_hash_status.txt"; else echo SOURCE_CHANGED > "$ART/source_hash_status.txt"; fi
printf 'EXIT_STATUS=%s\n' "$status" > "$ART/runner_status.txt"
tail -2 "$ART/pytest.log"
exit "$status"
