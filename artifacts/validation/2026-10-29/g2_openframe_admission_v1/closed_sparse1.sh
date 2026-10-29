#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -u
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_openframe_admission_v1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/$base${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$base/closed_sparse1.log" || exit 73
python3 -B -m pytest -q tests/test_bypass_handoff.py tests/test_bypass_entry_follow.py tests/test_bypass_entry_trial.py tests/test_follow_trace.py "$base/test_probe_shape.py" > "$base/sparse_tests.log" 2>&1
result=$?
echo "$result" > "$base/sparse_tests.exit"
if [ "$result" -ne 0 ]; then exit "$result"; fi
python3 -B "$base/probe_shape.py" --output closed_sparse1 --cases right_ideal right_combined left_combined --policies observed_entry_space_v1 --trace-policy observed_entry_space_v1 --window trim_before_compare_v1 --reserve --reacquire known_hold_v1 --patch-policy sparse_returns_v1 > "$base/closed_sparse1.log" 2>&1
result=$?
echo "$result" > "$base/closed_sparse1.exit"
exit "$result"
