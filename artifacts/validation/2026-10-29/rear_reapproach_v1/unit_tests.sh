#!/usr/bin/env bash
# 후보 단위 반례와 기존 선택기/연결 회귀. 인자로 로그 이름을 받는다(덮어쓰기 금지).
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_reapproach_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
name=${1:?log name required}
test ! -e "$base/$name.log"
shift
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$@" > "$base/$name.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/$name.exit"
exit "$result"
