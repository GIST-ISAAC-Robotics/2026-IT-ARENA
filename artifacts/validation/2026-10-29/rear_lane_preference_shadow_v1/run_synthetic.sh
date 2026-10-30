#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_lane_preference_shadow_v1
test ! -e "$base/synthetic1.log"
set +e
/usr/bin/python3 -B "$base/probe_candidate.py" --output "$base/synthetic1" --trace --cases A_recede_left B_reapproach_left B_late_reapproach_left C_lead_stop_left D_opposite_residual_left E_time_gap_left G_fast_lead_left > "$base/synthetic1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/synthetic1.exit"
exit "$rc"
