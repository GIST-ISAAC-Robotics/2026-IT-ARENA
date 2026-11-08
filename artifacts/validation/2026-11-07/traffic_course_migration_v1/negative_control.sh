#!/usr/bin/env bash
# scratch_check.sh 뒤에 실행: 같은 사본에 base/ 원본 스크립트를 되돌려 새 시험이 변경 전 코드를 잡는지 확인한다.
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A="$REPO/artifacts/validation/2026-11-07/traffic_course_migration_v1"
S="$HOME/arena_scratch/traffic_course_migration_v1"
cp "$A"/base/scripts/*.py "$S/scripts/" || exit 2
cd "$S" || exit 2
set +u; source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$S/src/arena_autonomy:$S/src/arena_vehicle_interface:$S/scripts:$S/tests"
nice -n 19 python3 -B -m pytest -q -p no:cacheprovider --tb=line tests/test_traffic_course_track.py > "$A/checks/negative_control_base.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "$A/checks/negative_control_base.exit"
cp "$A"/patched/scripts/*.py "$S/scripts/"
exit "$result"
