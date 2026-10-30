#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-30/g2_outward_replan_v1
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
step tests /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate.py"
step shadow /usr/bin/python3 -B "$base/shadow.py"
step figure /usr/bin/python3 -B artifacts/validation/2026-10-30/g2_center_width_v1/plot.py
