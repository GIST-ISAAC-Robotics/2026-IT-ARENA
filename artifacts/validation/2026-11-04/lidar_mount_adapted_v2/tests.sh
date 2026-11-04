#!/usr/bin/env bash
# 관련 회귀. 인자 없음. 결과는 tests.log/tests.exit.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-04/lidar_mount_adapted_v2
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B -m pytest -q -p no:cacheprovider \
  tests/test_known_structure_policy.py tests/test_partial_observation.py tests/test_traffic_recovery_contract.py tests/test_lidar_mount_trial.py tests/test_local_scene.py tests/test_local_sensor_profile.py tests/test_follow_trace.py \
  tests/test_local_pursuit.py tests/test_r0_safety_contract.py tests/test_recent_observation_safety.py tests/test_local_follow.py \
  tests/test_bypass_corridor_trial.py tests/test_observation_support.py tests/test_source_provenance_r0_plus_b3.py > "$A/tests.log" 2>&1
echo "ACTUAL_EXIT_CODE=$?" > "$A/tests.exit"
