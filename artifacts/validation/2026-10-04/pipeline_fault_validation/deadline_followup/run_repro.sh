#!/usr/bin/env bash
# ros_v2 실패 구간만 3회 반복한다. 실패해도 다음 회차를 계속하며 통과를 위해 추가 반복하지 않는다.
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
D=artifacts/validation/2026-10-04/pipeline_fault_validation/deadline_followup

for run in 1 2 3; do
    rc=0
    python3 scripts/validate_pipeline_faults_ros.py --output "$D/repro_r$run" --scenario restart_repro --cycles 1 \
        --sleep-wall .01 --wall-cap 600 > "$D/repro_r${run}_console.log" 2>&1 || rc=$?
    echo "run=$run exit=$rc" | tee -a "$D/repro_exit_codes.txt"
done
