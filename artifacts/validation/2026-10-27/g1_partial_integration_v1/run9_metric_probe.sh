#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_partial_integration_v1
test ! -e "$base/run9_metric_probe.log"
/usr/bin/python3 -B -m pytest -q tests/test_partial_lateral_metric.py tests/test_partial_observation.py tests/test_partial_surface_alignment.py > "$base/run9_metric_tests.log" 2>&1
/usr/bin/python3 -B "$base/run9_metric_probe.py" > "$base/run9_metric_probe.log" 2>&1
tail -2 "$base/run9_metric_probe.log"
