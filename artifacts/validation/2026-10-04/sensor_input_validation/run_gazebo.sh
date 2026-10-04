#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
colcon build --base-paths src --packages-select arena_autonomy arena_bringup --symlink-install
python3 scripts/validate_basic_autonomy.py \
  --autonomy-mode local_pursuit --lidar-acquisition sequential --lidar-compensation both \
  --lidar-rate-hz 10 --control-rate-hz 50 --speed-profile local_fast --disable-speed-bump \
  --actuation-mode virtual_mcu --actuation-stop software_stop --shutdown-mode service \
  --render-backend wsl_nvidia --sensor-wall-timeout-s 30 --stop-wall-timeout-s 120 \
  --grid-slot 3 --target-progress-m 12 --max-sim-seconds 60 --wall-timeout-s 900 \
  --output artifacts/validation/2026-10-04/sensor_input_validation/gazebo_12m_v1
