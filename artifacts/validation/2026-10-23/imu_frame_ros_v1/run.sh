#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source build/install_audit_20261023_install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=194
export ROS_LOCALHOST_ONLY=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
evidence=artifacts/validation/2026-10-23/imu_frame_ros_v1
test ! -e "$evidence/exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B "$evidence/run.py" > "$evidence/run.log" 2>&1
