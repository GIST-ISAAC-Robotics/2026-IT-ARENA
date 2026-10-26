#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-25/m3_execution_review_v1
out="$base/run18_front_target_review"
test ! -e "$out" || exit 91
mkdir -p "$out"
cp scripts/traffic_front_target.py tests/test_traffic_front_target.py "$out/"
python3 -B -m pytest -q tests/test_traffic_front_target.py tests/test_g1_traffic_eval.py tests/test_traffic_encounter.py > "$out/tests.txt" 2>&1
rc=$?
echo "$rc" > "$out/tests_exitcode.txt"
cat "$out/tests.txt"
if [ "$rc" -ne 0 ]; then exit "$rc"; fi
for name in run17_observed_release_closed_loop run13_selection_closed_loop; do
  python3 -B scripts/traffic_front_target.py "$base/$name/run" --output "$out/${name}_front.json" > "$out/${name}_stdout.txt" 2> "$out/${name}_stderr.txt"
  rc=$?
  echo "$rc" > "$out/${name}_exitcode.txt"
  cat "$out/${name}_stdout.txt"
  cat "$out/${name}_stderr.txt"
  if [ "$rc" -ne 0 ]; then exit "$rc"; fi
done
