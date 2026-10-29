#!/usr/bin/env bash
# 왼쪽 가장자리 시험의 설정·launch 사본 생성과 사전 검사(주행 없음).
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_entry_left_v1
/usr/bin/python3 -B "$base/prepare_fixture.py" > "$base/prepare_fixture.log" 2>&1; echo $? > "$base/prepare_fixture.exit"
/usr/bin/python3 -B "$base/preflight.py" > "$base/preflight.log" 2>&1; echo $? > "$base/preflight.exit"
