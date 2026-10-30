#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_lane_preference_shadow_v3
test ! -e "$base/tests.log"
set +e
PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate.py" > "$base/tests.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/tests.exit"
[ "$rc" = 0 ] || exit "$rc"
for profile in ideal combined; do
  cases='B_gazebo_left B_gazebo_right'
  if [ "$profile" = ideal ]; then cases="$cases G_fast_lead_left"; else cases="$cases C_lead_stop_left D_opposite_residual_left E_time_gap_left"; fi
  /usr/bin/python3 -B "$base/followup_probe.py" --policy candidate --sensor-profile "$profile" --output "$base/comparison_$profile" --trace --cases $cases > "$base/comparison_$profile.log" 2>&1
  rc=$?
  printf '%s\n' "$rc" > "$base/comparison_$profile.exit"
  [ "$rc" = 0 ] || exit "$rc"
done
