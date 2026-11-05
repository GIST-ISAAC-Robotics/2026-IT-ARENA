#!/usr/bin/env bash
# 격리 사본 환경에서 저장 판단 재생 진단. 사용: diag.sh <run 이름> <출력 이름> <시각...>
MAIN="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="$MAIN/artifacts/validation/2026-11-05/right_angle_course_v1"
WS="${WS_DIR:-$HOME/arena_scratch/right_angle_v1/ws}"
N="${1:?run}"; O="${2:?out}"; shift 2
cd "$WS" || exit 2
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$A/diag_replay.py" "$A/$N" "$A/$O.json" "$@" > "$A/$O.log" 2>&1
echo "DIAG_EXIT=$?" >> "$A/$O.log"
