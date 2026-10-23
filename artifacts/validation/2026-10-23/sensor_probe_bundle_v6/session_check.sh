#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_bundle_v6
test ! -e "$evidence/session_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/session_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B "$evidence/portable_session.py" > "$evidence/portable_session.log" 2>&1
