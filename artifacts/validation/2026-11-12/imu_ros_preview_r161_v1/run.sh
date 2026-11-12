#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_imu_preview_r161_20261112
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=191
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-12/imu_ros_preview_r161_v1/run.py
