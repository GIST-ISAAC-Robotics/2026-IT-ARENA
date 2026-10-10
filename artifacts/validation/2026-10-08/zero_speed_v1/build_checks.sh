#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base='artifacts/validation/2026-10-08/zero_speed_v1'
test ! -e "$base/preflight.log"
python3 -B -m pytest -q -p no:cacheprovider tests/test_zero_speed_modes.py tests/test_drivetrain_model.py \
  tests/test_wheel_contact_probe.py tests/test_braking_command_profile.py tests/test_braking_ab_analysis.py \
  > "$base/preflight.log" 2>&1
test ! -e "$base/build.log"
colcon build --packages-select arena_gazebo arena_bringup --symlink-install > "$base/build.log" 2>&1
ctest --test-dir build/arena_gazebo --output-on-failure > "$base/ctest.log" 2>&1
sha256sum install/arena_gazebo/lib/libarena-single-motor-system.so > "$base/plugin.sha256"
