#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
# 지난 579개 집합과 같은 제외 범위. 실행/종료 검증은 이번 ROS/Gazebo 결과와 구분한다.
python3 -m pytest tests src/arena_vehicle_interface/test -q \
  --ignore=tests/test_lidar_shutdown.py --ignore=tests/test_local_pursuit.py \
  --ignore=tests/test_replay_imu_probe.py --ignore=tests/test_replay_observer.py \
  --ignore=tests/test_wsl_d3d12_lifetime.py
