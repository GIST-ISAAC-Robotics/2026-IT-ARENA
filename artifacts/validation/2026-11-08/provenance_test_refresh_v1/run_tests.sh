#!/usr/bin/env bash
# 사용: run_tests.sh <label> <pytest args...>  (WSL ROS 환경, 시뮬레이터 미실행)
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
label="$1"; shift
out=artifacts/validation/2026-11-08/provenance_test_refresh_v1/checks/pytest_${label}
python3 -B -m pytest -q -p no:cacheprovider --tb=short "$@" > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"
