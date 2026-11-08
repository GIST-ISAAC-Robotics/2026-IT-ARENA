#!/usr/bin/env bash
# 사용: run_tests.sh <label> <test files...>   (WSL ROS 환경 pytest. 시뮬레이터 미사용. 명령·종료 코드 기록)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
A=artifacts/validation/2026-11-08/corner_path_anchor_v1
label="$1"; shift
python3 -B -m pytest -q -p no:cacheprovider --tb=short "$@" > "$A/pytest_${label}.log" 2>&1
result=$?
echo "COMMAND=python3 -B -m pytest -q -p no:cacheprovider --tb=short $* EXIT=$result" > "$A/pytest_${label}.exit.txt"
exit "$result"
