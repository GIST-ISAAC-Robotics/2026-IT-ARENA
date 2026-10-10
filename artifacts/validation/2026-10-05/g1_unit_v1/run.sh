#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out='artifacts/validation/2026-10-05/g1_unit_v1'
test ! -e "$out/pytest.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_local_follow.py >"$out/pytest.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" >"$out/returncode"
tail -55 "$out/pytest.log"
exit "$code"
