#!/usr/bin/env bash
# r7 관련 정적 회귀(주행 없음). 11/4 jr60 채택 회귀 묶음 + 추종/경계/측정 출처. 결과 <이름>.log/<이름>.exit.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-05/lidar_structure_review_v1
N="${1:-tests}"
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh /usr/bin/python3 -B -m pytest -q -p no:cacheprovider \
  tests/test_jr60_adoption_profile.py tests/test_g1_traffic_extensions.py tests/test_known_structure_policy.py tests/test_structure_route_speed.py tests/test_lidar_mount_trial.py \
  tests/test_local_sensor_profile.py tests/test_partial_observation.py tests/test_local_pursuit.py tests/test_r0_safety_contract.py \
  tests/test_source_provenance_r0_plus_b3.py tests/test_measured_lane_preference_provenance.py tests/test_local_follow.py tests/test_boundary_bridge.py > "$A/$N.log" 2>&1
echo "ACTUAL_EXIT_CODE=$?" > "$A/$N.exit"
