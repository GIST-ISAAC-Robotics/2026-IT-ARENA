#!/usr/bin/env bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
printf '%s\n' '$ python3 -m pytest -v (gearing and signed-forward regression set)'
python3 -m pytest -v \
  tests/test_drivetrain_model.py tests/test_drive_feedback.py tests/test_bypass_probe_sensors.py \
  tests/test_actuation_conformance.py tests/test_motor_encoder_runtime_audit.py tests/test_bag_contract.py \
  tests/test_grid_departure_dynamics.py tests/test_grid_departure_impairment.py \
  tests/test_signed_forward_drive.py tests/test_local_follow.py tests/test_partial_observation.py \
  tests/test_recent_observation_safety.py tests/test_joint_missing_policy_probe.py tests/test_lidar_safety.py \
  tests/test_bypass_handoff.py tests/test_bypass_signed_hold_trial.py \
  tests/test_bypass_motion_pause_connection.py tests/test_follow_trace.py \
  tests/test_source_provenance_r0_plus_b3.py 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/build_checks/pytest_regression.log
rc=${PIPESTATUS[0]}
printf 'ACTUAL_EXIT_CODE=%s\n' "$rc" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/build_checks/pytest_regression.log
exit "$rc"
