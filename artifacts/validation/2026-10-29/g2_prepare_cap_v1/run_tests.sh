#!/usr/bin/env bash
# 준비 감속 묶음의 관련 회귀. 인수: 로그 이름(덮어쓰기 금지), 이후 pytest 대상.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_prepare_cap_v1
name="$1"; shift
test ! -e "$base/$name.log" || exit 73
python3 -B -m pytest -q -p no:cacheprovider "$@" > "$base/$name.log" 2>&1
result=$?
echo "$result" > "$base/$name.exit"
exit "$result"
