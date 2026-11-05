#!/usr/bin/env bash
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
T=$(python3 -c "print(' '.join(f'{17.2+0.02*i:.2f}' for i in range(20))+' 20.30 20.32 20.34 20.36 20.38')")
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" bash "$D/diag.sh" run6 trial_patch/run6_diag_b $T
