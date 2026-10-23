#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_bundle_v6
test ! -e "$evidence/runtime_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/runtime_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B "$evidence/runtime_check.py" > "$evidence/runtime.log" 2>&1
