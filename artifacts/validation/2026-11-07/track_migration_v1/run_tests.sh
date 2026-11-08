#!/usr/bin/env bash
# 사용: run_tests.sh <label> <test files...>
# 이관 영향 범위(빌더·실행기·출처 계약 참조) 테스트를 WSL ROS 환경에서 실행합니다.
# 시뮬레이터는 띄우지 않습니다. PYTHONPATH는 11/2 official_facility_v1/full_tests.sh와 같습니다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
label="$1"; shift
out=artifacts/validation/2026-11-07/track_migration_v1/checks/pytest_${label}
python3 -B -m pytest -q -p no:cacheprovider --tb=line "$@" > "${out}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "${out}.exit"
exit "$result"
