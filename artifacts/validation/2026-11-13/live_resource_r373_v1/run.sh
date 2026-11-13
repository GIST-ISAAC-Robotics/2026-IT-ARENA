set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=216
export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
/usr/bin/python3 -c "import serial, rclpy"
python3 /home/q/r373b_resources.py
