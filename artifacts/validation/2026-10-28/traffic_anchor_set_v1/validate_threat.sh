#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider tests/test_recovery_threat_capture.py tests/test_traffic_recovery_contract.py tests/test_surface_anchor_probe.py tests/test_timed_policy_probe.py tests/test_partial_observation.py > artifacts/validation/2026-10-28/traffic_anchor_set_v1/threat_regression_v3.log 2>&1
