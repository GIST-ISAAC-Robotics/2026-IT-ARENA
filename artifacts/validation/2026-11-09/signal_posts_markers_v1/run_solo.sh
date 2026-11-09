#!/usr/bin/env bash
# 사용: run_solo.sh <label> [run_new_course_solo 추가 인자...]
# 지주/마커 묶음의 단독 1랩(r25 프리셋). 기존 결과 경로는 거절되며 STOP·종료는 기존 검증기가 맡는다.
set -uo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
HERE="$ROOT/artifacts/validation/2026-11-09/signal_posts_markers_v1"
cd "$ROOT"
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
label="$1"; shift
sha256sum src/arena_bringup/launch/simulation.launch.py src/arena_bringup/launch/demo.launch.py \
  scripts/validate_basic_autonomy.py scripts/run_new_course_solo.py scripts/run_current_solo.py \
  src/arena_autonomy/arena_autonomy/local_follow.py > "$HERE/${label}.sources_before.txt"
python3 -B scripts/run_new_course_solo.py "$HERE/$label" --preset new-course-solo --check-r25 "$@" > "$HERE/${label}.check_r25.json" 2>&1
python3 -B scripts/run_new_course_solo.py "$HERE/$label" --preset new-course-solo "$@" > "$HERE/${label}.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "$HERE/${label}.exit"
sha256sum -c "$HERE/${label}.sources_before.txt" > "$HERE/${label}.sources_after.txt" 2>&1
exit "$result"
