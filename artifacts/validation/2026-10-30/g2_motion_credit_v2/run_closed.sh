#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-30/g2_motion_credit_v2
step() {
  local name=$1; shift
  test ! -e "$base/$name.log"
  set +e
  "$@" > "$base/$name.log" 2>&1
  local rc=$?
  set -e
  printf '%s\n' "$rc" > "$base/$name.exit"
  [ "$rc" = 0 ] || exit "$rc"
}
for variant in motion_v2; do
  step "closed_${variant}" /usr/bin/python3 -B "$base/probe_closed_loop.py" --variant "$variant" --output "closed_${variant}" --cases right_ideal moving_combined stopping_combined stop_restart_combined --policies observed_entry_space_v1 --trace-policy observed_entry_space_v1 --window trim_before_compare_v1 --reserve --reacquire known_hold_v1 --patch-policy sparse_returns_v1
  step "static_${variant}" /usr/bin/python3 -B "$base/probe_closed_loop.py" --variant "$variant" --output "static_${variant}" --cases right_combined --duration 16 --policies observed_entry_space_v1 --trace-policy observed_entry_space_v1 --window trim_before_compare_v1 --reserve --reacquire known_hold_v1 --patch-policy sparse_returns_v1
done
