#!/usr/bin/env bash
# 사용: run_tests.sh <label> <pytest args...>
# WSL ROS 환경에서 pytest만 실행한다(시뮬레이터·빌드 없음, python3 -B로 pyc 미생성).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
label="$1"; shift
out=artifacts/validation/2026-11-08/g1g2_lidar_stress_pair_v1/checks/pytest_${label}
python3 -B -m pytest -q -p no:cacheprovider -rfE --tb=short "$@" > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"
