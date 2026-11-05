#!/usr/bin/env bash
# 후보 v2로 ws2 재생성 후 run3(v1 폐루프) 저장 입력을 v2 코드로 재생(평가 전용).
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
W2="$HOME/arena_scratch/right_angle_v1/ws2"
case "$W2" in "$HOME"/arena_scratch/right_angle_v1/ws2) rm -rf "$W2";; *) exit 4;; esac
bash "$D/trial_patch/setup_ws2.sh" course_world > "$D/trial_patch/setup_ws2.out" 2>&1
echo "SETUP_WS2_EXIT=$?" >> "$D/trial_patch/setup_ws2.out"
WS_DIR="$W2" bash "$D/diag.sh" run4 trial_patch/run4_replay_v3 13.42 13.52 13.62 13.72 13.82 14.02 16.02
