#!/usr/bin/env bash
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
logs='artifacts/validation/2026-10-05/g1_runner_v4'
/usr/bin/python3 -B scripts/validate_local_follow.py --output artifacts/validation/2026-10-05/g1_synthetic_v4 > "$logs/synthetic.stdout.log" 2> "$logs/synthetic.stderr.log"
rc=$?
printf '%s\n' "$rc" > "$logs/synthetic.exitcode"
tail -8 "$logs/synthetic.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
