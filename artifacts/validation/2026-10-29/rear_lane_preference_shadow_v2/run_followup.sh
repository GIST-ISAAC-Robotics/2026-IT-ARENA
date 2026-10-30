#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/rear_lane_preference_shadow_v2
for policy in current candidate; do
  for profile in ideal combined; do
    name="followup_${policy}_${profile}"
    test ! -e "$base/$name.log"
    set +e
    /usr/bin/python3 -B "$base/followup_probe.py" --policy "$policy" --sensor-profile "$profile" --output "$base/$name" --trace --cases B_gazebo_left B_gazebo_right > "$base/$name.log" 2>&1
    rc=$?
    printf '%s\n' "$rc" > "$base/$name.exit"
    set -e
    [ "$rc" = 0 ] || exit "$rc"
  done
done
