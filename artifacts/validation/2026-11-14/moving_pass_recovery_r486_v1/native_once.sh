#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/moving_pass_recovery_r486_v1'
mkdir -p "$artifact/native_v1"
echo 'r486 WSL entered; native limited regression only'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 -B "$artifact/run_native_regression.py" > "$artifact/native_v1/pytest.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$artifact/native_v1/run.exit"
tail -n 65 "$artifact/native_v1/pytest.log"
if test -f "$artifact/native_v1/run.pid"; then
    read -r owned_pid < "$artifact/native_v1/run.pid"
    if test -d "/proc/$owned_pid"; then
        printf 'owned pid still present: %s\n' "$owned_pid"
        exit 3
    fi
fi
printf 'owned_python_pid_remaining=0\n' > "$artifact/native_v1/resource_return.txt"
date --utc --iso-8601=seconds >> "$artifact/native_v1/resource_return.txt"
echo "r486 native regression completed exit=$code; owned Python PID remaining=0"
exit "$code"
