#!/usr/bin/env bash
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
logs='artifacts/validation/2026-10-05/g1_runner_v2'
set +e
/usr/bin/python3 -B scripts/validate_local_follow.py --output artifacts/validation/2026-10-05/g1_synthetic_v2 > "$logs/synthetic.stdout.log" 2> "$logs/synthetic.stderr.log"
rc=$?
set -e
printf '%s\n' "$rc" > "$logs/synthetic.exitcode"
tail -12 "$logs/synthetic.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
