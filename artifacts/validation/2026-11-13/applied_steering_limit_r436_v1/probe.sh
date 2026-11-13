#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
python3 artifacts/validation/2026-11-13/applied_steering_limit_r436_v1/ros_parameter_probe.py
