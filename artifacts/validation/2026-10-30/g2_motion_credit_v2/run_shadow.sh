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
step tests /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate.py" tests/test_bypass_handoff.py
step trace_boundary /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_trace_boundary.py"
step shadow_motion_v2 /usr/bin/python3 -B "$base/probe_course_shadow.py" --output shadow_motion_v2 --policy observed_entry_space_v1 --variant motion_v2
