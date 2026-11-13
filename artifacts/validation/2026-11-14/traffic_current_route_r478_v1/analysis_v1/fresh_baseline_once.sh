#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/traffic_current_route_r478_v1/analysis_v1'
echo 'r478 WSL entered; fresh baseline three inputs only'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
date --utc --iso-8601=seconds > "$artifact/fresh_baseline_started_utc.txt"
set +e
python3 -B "$artifact/fresh_baseline_wsl.py" > "$artifact/fresh_baseline_wsl.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$artifact/fresh_baseline.exit"
cat "$artifact/fresh_baseline_wsl.log"
if test -f "$artifact/fresh_baseline.pid"; then
    read -r owned_pid < "$artifact/fresh_baseline.pid"
    if test -d "/proc/$owned_pid"; then
        printf 'owned pid still present: %s\n' "$owned_pid"
        exit 3
    fi
fi
printf 'owned_python_pid_remaining=0\n' > "$artifact/fresh_baseline_resource_return.txt"
date --utc --iso-8601=seconds >> "$artifact/fresh_baseline_resource_return.txt"
echo "r478 fresh baseline completed exit=$code; owned Python PID remaining=0"
exit "$code"
