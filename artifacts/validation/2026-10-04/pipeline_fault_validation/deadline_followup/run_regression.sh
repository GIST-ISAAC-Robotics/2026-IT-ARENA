#!/usr/bin/env bash
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
D=artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup

python3 -m pytest -v -rs -p no:cacheprovider tests/test_steering_transport_limit.py tests/test_actuation_contract.py \
    tests/test_actuation_ros_nodes.py tests/test_pipeline_fault_validation.py > "$D/regression_after.log" 2>&1
tail -n 6 "$D/regression_after.log"
