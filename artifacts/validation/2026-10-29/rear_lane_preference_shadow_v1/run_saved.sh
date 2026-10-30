#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/rear_lane_preference_shadow_v1
test ! -e "$base/tests.log"
set +e
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider "$base/test_candidate.py" > "$base/tests.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/tests.exit"
[ "$rc" = 0 ] || exit "$rc"
for variant in off on; do
 /usr/bin/python3 -B "$base/compare_saved.py" "artifacts/validation/2026-10-29/rear_reapproach_gazebo_v2/drive_${variant}/follow_decisions.jsonl" --output "$base/shadow_${variant}.json" > "$base/shadow_${variant}.log" 2>&1
 rc=$?
 printf '%s\n' "$rc" > "$base/shadow_${variant}.exit"
 [ "$rc" = 0 ] || exit "$rc"
done
