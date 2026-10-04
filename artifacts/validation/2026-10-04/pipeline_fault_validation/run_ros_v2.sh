#!/usr/bin/env bash
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"

python3 -m pytest -q tests/test_pipeline_fault_validation.py 2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/unit_v2.log
python3 scripts/validate_pipeline_faults_ros.py --output artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v2 --sleep-wall .01 --wall-cap 600 2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/ros_v2_console.log
