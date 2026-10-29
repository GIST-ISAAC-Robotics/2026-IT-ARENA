#!/usr/bin/env bash
# 이 디렉터리의 비교 스크립트를 ROS 환경에서 실행한다. 인수: 로그 이름(덮어쓰기 금지), 스크립트, 인수...
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_prepare_cap_v1
name="$1"; script="$2"; shift 2
test ! -e "$base/$name.log" || exit 73
python3 -B "$base/$script" "$@" > "$base/$name.log" 2>&1
result=$?
echo "$result" > "$base/$name.exit"
exit "$result"
