#!/usr/bin/env bash
# 단계 1: v4 원본 경계 반례 재현 → 수정본 반례/기존 37건 이식 검사.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/rear_lane_measured_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
tag=${1:?run tag required}
test ! -e "$base/boundary_$tag.log"
set +e
{
  echo '## v4 original reproduction'
  /usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_v4_boundary_reproduction.py"
  echo "rc_original=$?"
  echo '## runtime successor'
  /usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_measured_lane_preference_trial.py
  echo "rc_successor=$?"
} > "$base/boundary_$tag.log" 2>&1
grep -q '^rc_original=0$' "$base/boundary_$tag.log" && grep -q '^rc_successor=0$' "$base/boundary_$tag.log"
rc=$?
printf '%s\n' "$rc" > "$base/boundary_$tag.exit"
exit "$rc"
