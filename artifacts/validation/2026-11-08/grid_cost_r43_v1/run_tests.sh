#!/usr/bin/env bash
# 사용: run_tests.sh <label> <test files...>  — r43 회귀를 WSL에서 실행(시뮬레이터 없음).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
label="$1"; shift
out=artifacts/validation/2026-11-08/grid_cost_r43_v1/checks/pytest_${label}
python3 -B -m pytest -q -p no:cacheprovider --tb=short "$@" > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"
