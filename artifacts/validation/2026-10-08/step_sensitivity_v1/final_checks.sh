#!/usr/bin/env bash
# 마감 회귀(직전 0속도 마감 목록 + 이번 step 옵션 시험)와 플러그인 불변 확인.
set -e
source "$(dirname "$0")/env.sh"
name="${1:-pytest_final.log}"
case "$name" in pytest_final.log|pytest_final_v2.log) ;; *) exit 2;; esac
test ! -e "$base/$name"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_physics_step_option.py \
  tests/test_zero_speed_modes.py tests/test_zero_speed_analysis.py tests/test_drivetrain_model.py \
  tests/test_braking_command_profile.py tests/test_braking_ab_analysis.py tests/test_g1_side_occlusion.py \
  tests/test_g1_traffic_extensions.py tests/test_local_follow.py tests/test_g1_traffic_eval.py \
  tests/test_local_follow_validation.py tests/test_local_scene.py tests/test_g0_geometry.py \
  tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py tests/test_local_pursuit.py \
  tests/test_lidar_safety.py tests/test_lidar_observation.py tests/test_lidar_motion.py \
  tests/test_rotating_lidar.py tests/test_lidar_motion_ros.py tests/test_vision_process.py \
  tests/test_g1_traffic_coverage_audit.py tests/test_local_path_range_noise.py tests/test_official_track.py \
  tests/test_runtime_world_options.py tests/test_experimental_facilities.py tests/test_track_variants.py \
  tests/test_g1b_braking_analysis.py tests/test_wheel_contact_probe.py tests/test_wheel_contact_analysis.py \
  tests/test_drive_feedback.py tests/test_motor_encoder_validation.py tests/test_actuation_contract.py \
  > "$base/$name" 2>&1
rc=$?
sha256sum -c artifacts/validation/2026-10-08/zero_speed_v1/plugin.sha256 > "$base/plugin_after.txt" 2>&1
echo "pytest_rc=$rc plugin=$(cat "$base/plugin_after.txt")"
exit $rc
