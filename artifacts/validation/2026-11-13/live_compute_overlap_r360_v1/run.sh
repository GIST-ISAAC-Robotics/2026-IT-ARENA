#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it-arena-r358-20261113
source /opt/ros/humble/setup.bash
source /home/q/it_arena_c1_20261110/install/setup.bash
export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 -B artifacts/validation/2026-11-13/live_compute_overlap_r360_v1/bench.py
