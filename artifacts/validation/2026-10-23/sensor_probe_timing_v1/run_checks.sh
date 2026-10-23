#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_timing_v1
test ! -e "$evidence/checks_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/checks_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B "$evidence/check.py"
