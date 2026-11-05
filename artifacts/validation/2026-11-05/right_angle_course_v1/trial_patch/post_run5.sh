#!/usr/bin/env bash
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
WORLD=course_world bash "$D/post.sh" run5
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" bash "$D/diag.sh" run5 trial_patch/run5_diag 14.02 14.12 14.22 14.32 14.42 17.02
