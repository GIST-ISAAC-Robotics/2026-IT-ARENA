#!/usr/bin/env bash
set +e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-07/return_guide_front_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 "$A/replay_connected.py" > "$A/replay_connected.log" 2>&1
run_status=$?
printf "ACTUAL_EXIT_CODE=%s\\n" "$run_status" > "$A/replay_connected.exit"
exit "$run_status"
