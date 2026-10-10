#!/usr/bin/env bash
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
seed="$1"
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out="artifacts/validation/2026-10-05/g1_synthetic_v1_seed${seed}"
logs="artifacts/validation/2026-10-05/g1_runner_v1"
set +e
/usr/bin/python3 -B scripts/validate_local_follow.py --seed "$seed" --output "$out" > "$logs/seed${seed}.stdout.log" 2> "$logs/seed${seed}.stderr.log"
rc=$?
set -e
printf '%s\n' "$rc" > "$logs/seed${seed}.returncode"
tail -8 "$logs/seed${seed}.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
