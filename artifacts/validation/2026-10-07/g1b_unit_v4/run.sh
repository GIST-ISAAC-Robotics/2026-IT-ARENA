#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out='artifacts/validation/2026-10-07/g1b_unit_v4'
test ! -e "$out/pytest.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_local_follow.py tests/test_g1_traffic_eval.py tests/test_local_follow_validation.py tests/test_local_scene.py tests/test_g0_geometry.py tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py tests/test_local_pursuit.py tests/test_lidar_safety.py tests/test_lidar_observation.py tests/test_lidar_motion.py tests/test_rotating_lidar.py tests/test_lidar_motion_ros.py tests/test_vision_process.py >"$out/pytest.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" >"$out/returncode"
tail -70 "$out/pytest.log"
exit "$code"
