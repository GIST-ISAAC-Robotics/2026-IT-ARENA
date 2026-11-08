#!/usr/bin/env bash
# 같은 scratch 사본에 base/ 원본 4개 스크립트를 넣고 같은 관련 시험을 돌려 기존 실패를 분리한다.
# 이후 patched/ 사본을 되돌린다. 사용: baseline_related.sh <tests...>
REPO='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A="$REPO/artifacts/validation/2026-11-07/traffic_course_migration_v1"
S="$HOME/arena_scratch/traffic_course_migration_v1"
cp "$A"/base/scripts/*.py "$S/scripts/" || exit 2
cd "$S" || exit 2
set +u; source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$S/src/arena_autonomy:$S/src/arena_vehicle_interface:$S/scripts:$S/tests"
nice -n 19 python3 -B -m pytest -q -p no:cacheprovider --tb=line -rfE "$@" > "$A/checks/related_base.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "$A/checks/related_base.exit"
cp "$A"/patched/scripts/*.py "$S/scripts/"
nice -n 19 python3 -B -m pytest -q -p no:cacheprovider --tb=line -rfE "$@" > "$A/checks/related_patched_rf.log" 2>&1
echo "ACTUAL_EXIT_CODE=$?" > "$A/checks/related_patched_rf.exit"
exit "$result"
