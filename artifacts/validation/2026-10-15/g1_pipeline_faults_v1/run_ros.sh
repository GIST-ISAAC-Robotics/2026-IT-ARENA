#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_pipeline_faults_v1
test ! -e "$base/ros"
test ! -e "$base/runner.log"
if (( $(date -u +%s) >= 1792002600 )); then
  echo wrap_up_deadline_reached_before_execution
  exit 0
fi
set +e
python3 -B scripts/validate_pipeline_faults_ros.py --output "$base/ros" --scenario full --controller-profile g1_per_ray_candidate --sleep-wall .01 --wall-cap 600 > "$base/runner.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$base/exit.txt"
tail -6 "$base/runner.log"
exit "$code"
