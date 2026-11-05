#!/usr/bin/env bash
# 시험 후보 수정 뒤 ws2를 지우고 다시 만든다(ws2는 격리 시험 전용 scratch).
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
W2="$HOME/arena_scratch/right_angle_v1/ws2"
case "$W2" in "$HOME"/arena_scratch/right_angle_v1/ws2) rm -rf "$W2";; *) exit 4;; esac
bash "$D/trial_patch/setup_ws2.sh" "${1:-course_world}" > "$D/trial_patch/setup_ws2.out" 2>&1
echo "SETUP_WS2_EXIT=$?" >> "$D/trial_patch/setup_ws2.out"
sed -i "s#run1 trial_patch/[a-z0-9_]* 12.72#run1 trial_patch/${2:-run1_replay_ws2_v3} 12.72#" "$D/trial_patch/diag_ws2.sh"
bash "$D/trial_patch/diag_ws2.sh"
