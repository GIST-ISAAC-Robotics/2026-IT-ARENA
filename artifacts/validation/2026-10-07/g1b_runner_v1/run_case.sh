#!/usr/bin/env bash
# 사용: run_case.sh <case> <seed> <g1|r0> <output-dir-under-artifacts>
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
case_name="$1"; seed="$2"; mode="$3"; out="$4"
logs="artifacts/validation/2026-10-07/g1b_runner_v1/logs"
mkdir -p "$logs"
tag="$(basename "$out")"
test ! -e "$out" || { echo "output exists: $out"; exit 93; }
/usr/bin/python3 -B scripts/validate_g1_traffic.py --case "$case_name" --seed "$seed" --mode "$mode" --output "$out" \
  > "$logs/$tag.stdout.log" 2> "$logs/$tag.stderr.log"
rc=$?
printf '%s\n' "$rc" > "$logs/$tag.exitcode"
tail -40 "$logs/$tag.stdout.log"
printf 'returncode=%s\n' "$rc"
exit "$rc"
