#!/usr/bin/env bash
# 10/29 신호 시험본 정지 촬영 4회(official/no_id45 x red4/green). 제어기·MCU·주행 프로세스 없음.
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 2
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
BASE=artifacts/validation/2026-10-29/signal_spec_1029_v1
export PYTHONPATH="$PWD/scripts:${PYTHONPATH:-}"
for name in official_red4 official_green no_id45_red4 no_id45_green; do
  /usr/bin/python3 -B artifacts/validation/2026-10-27/official_signal_orientation_v1/capture.py "$BASE/render_$name" --timeout 240 \
    > "$BASE/capture_$name.log" 2>&1
  echo $? > "$BASE/capture_$name.exit"
  echo "$name exit $(cat "$BASE/capture_$name.exit")"
done
