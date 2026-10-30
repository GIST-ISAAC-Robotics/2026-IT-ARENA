#!/usr/bin/env bash
# WSL에서 ROS 환경을 불러 합성 평가기를 실행한다. 인자는 python3에 그대로 전달.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -B "$@"
