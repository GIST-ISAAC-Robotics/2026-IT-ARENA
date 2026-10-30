#!/usr/bin/env bash
# 중앙 표적 시험의 사전 검사와 실행 입력 고정(주행 없음).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_center_gazebo_v1
/usr/bin/python3 -B "$base/preflight.py" > "$base/preflight.log" 2>&1; echo $? > "$base/preflight.exit"
/usr/bin/python3 -B "$base/run.py" --output "$PWD/$base/run1" --prepare-only > "$base/prepare.log" 2>&1; echo $? > "$base/prepare.exit"
