#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-10/observed_entry_return_r63_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B -m pytest -q tests/test_bypass_observed_return.py \
 > "$A/tests_v3.log" 2>&1
code=$?; printf '%s\n' "$code" > "$A/tests_v3.exit"
if (( code != 0 )); then exit "$code"; fi
bash "$ENV" /usr/bin/python3 -B "$A/replay.py" --current-only > "$A/replay_v2.log" 2>&1
code=$?; printf '%s\n' "$code" > "$A/replay_v2.exit"
exit "$code"
