#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/home/q/it-arena-r447-a/src/arena_autonomy:/home/q/it-arena-r447-a/src/arena_vehicle_interface:/home/q/it-arena-r447-a:${PYTHONPATH:-}"
python3 -B /home/q/it-arena-r447-node-only.py
