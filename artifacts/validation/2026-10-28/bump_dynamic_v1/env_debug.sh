#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
printf 'AFTER_ROS=%s\n' "${ROS_DISTRO-unset}"
printf 'ROS_PYTHONPATH=%s\n' "${PYTHONPATH-unset}"
source install/local_setup.bash
printf 'AFTER_INSTALL=%s\n' "${ROS_DISTRO-unset}"
printf 'INSTALL_PYTHONPATH=%s\n' "${PYTHONPATH-unset}"
/usr/bin/python3 -B -c 'import rclpy; print(rclpy.__file__)'
