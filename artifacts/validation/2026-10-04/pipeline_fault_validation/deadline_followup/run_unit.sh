#!/usr/bin/env bash
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
D=artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup

python3 -m pytest -q -p no:cacheprovider tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py \
    tests/test_pipeline_fault_validation.py 2>&1 | tee "$D/unit.log"
sha256sum src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py \
    src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py \
    scripts/validate_pipeline_faults_ros.py tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py \
    | tee "$D/after_change_sha256.txt"
