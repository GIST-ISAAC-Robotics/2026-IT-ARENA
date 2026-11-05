#!/usr/bin/env bash
# ws2(시험 후보) 코드로 run1 저장 입력 재생(평가 전용). 기록 출력과의 불일치는 후보 변경의 영향이다.
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" bash "$D/diag.sh" run1 trial_patch/run1_replay_ws2_v3 12.72 13.02 13.12 13.22 13.32 13.42
