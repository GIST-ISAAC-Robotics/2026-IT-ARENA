#!/usr/bin/env bash
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out='artifacts/validation/2026-10-05/g1_validator_v1'
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_local_follow_validation.py > "$out/pytest.stdout.log" 2> "$out/pytest.stderr.log"
rc=$?
set -e
printf '%s\n' "$rc" > "$out/pytest.exitcode"
tail -15 "$out/pytest.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
