#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_writer_faults_v2
test ! -e "$evidence/run_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/run_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B "$evidence/run.py" > "$evidence/run.log" 2>&1
