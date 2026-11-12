#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_sensor_link_ingress_r240_20261112/worktree
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-12/sensor_link_ingress_r240_v1/run.py
