#!/usr/bin/env bash
set +e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-07/paired_prefix_v1
start=$(date +%s)
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/replay_shadow.py" > "$A/replay_shadow.log" 2>&1
run_status=$?
printf "ACTUAL_EXIT_CODE=%s WALL_S=%s\\n" "$run_status" "$(( $(date +%s) - start ))" > "$A/replay_shadow.exit"
exit "$run_status"
