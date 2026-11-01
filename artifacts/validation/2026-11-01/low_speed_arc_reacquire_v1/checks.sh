#!/usr/bin/env bash
# 관련 회귀 실행. 인자: 로그 이름. 빌드·Gazebo는 실행하지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:$PYTHONPATH"
export PYTHONDONTWRITEBYTECODE=1
A=artifacts/validation/2026-11-01/low_speed_arc_reacquire_v1
FILES=$(grep -lE "local_follow|FollowConfig|LocalFollow|validate_basic_autonomy|simulation\.launch|SPEED_PROFILES|local_pursuit|follow_trace" tests/*.py)
nice -n 10 python3 -m pytest -q -p no:cacheprovider --tb=line $FILES > "$A/$1" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" >> "$A/$1"
tail -28 "$A/$1" | cut -c1-200
exit $code
