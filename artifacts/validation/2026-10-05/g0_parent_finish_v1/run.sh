#!/usr/bin/env bash
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
runner='artifacts/validation/2026-10-05/g0_parent_finish_v1'
for case_name in stop_restart low_target rear_mask short_branch_wall; do
  set +e
  /usr/bin/python3 scripts/validate_multi_vehicle_g0.py --mode gazebo --case "$case_name" --render-backend wsl_nvidia --output "artifacts/validation/2026-10-05/multi_vehicle_g0_finish_v1/$case_name" >"$runner/$case_name.stdout.log" 2>"$runner/$case_name.stderr.log"
  case_status=$?
  set -e
  printf '%s\n' "$case_status" >"$runner/$case_name.returncode"
  printf '%s returncode=%s\n' "$case_name" "$case_status"
  if [ "$case_status" -ne 0 ]; then
    exit "$case_status"
  fi
done
