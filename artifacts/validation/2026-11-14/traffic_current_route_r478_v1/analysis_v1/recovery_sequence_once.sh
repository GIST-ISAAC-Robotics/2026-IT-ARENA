#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/traffic_current_route_r478_v1/analysis_v1'
echo 'r478 WSL entered; recovery candidate five sequential inputs only'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
date --utc --iso-8601=seconds > "$artifact/recovery_sequence_started_utc.txt"
set +e
python3 -B "$artifact/recovery_sequence_wsl.py" > "$artifact/recovery_sequence_wsl.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$artifact/recovery_sequence.exit"
cat "$artifact/recovery_sequence_wsl.log"
if test -f "$artifact/recovery_sequence.pid"; then
    read -r owned_pid < "$artifact/recovery_sequence.pid"
    if test -d "/proc/$owned_pid"; then
        printf 'owned pid still present: %s\n' "$owned_pid"
        exit 3
    fi
fi
printf 'owned_python_pid_remaining=0\n' > "$artifact/recovery_sequence_resource_return.txt"
date --utc --iso-8601=seconds >> "$artifact/recovery_sequence_resource_return.txt"
echo "r478 recovery sequence completed exit=$code; owned Python PID remaining=0"
exit "$code"
