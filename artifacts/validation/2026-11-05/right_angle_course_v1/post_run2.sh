#!/usr/bin/env bash
# run2(iso_world) 회수·재생 진단 후 ws2(격리 시험 후보) 준비.
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
WORLD=iso_world bash "$D/post.sh" run2
bash "$D/diag.sh" run2 run2_diag 13.2 13.26 13.32 13.42
bash "$D/trial_patch/setup_ws2.sh" course_world > "$D/trial_patch/setup_ws2.out" 2>&1
echo "SETUP_WS2_EXIT=$?" >> "$D/trial_patch/setup_ws2.out"
