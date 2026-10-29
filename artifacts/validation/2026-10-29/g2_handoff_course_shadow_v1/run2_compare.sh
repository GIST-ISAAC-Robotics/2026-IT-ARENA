#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_handoff_course_shadow_v1
test ! -e "$base/run2" || exit 73
python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass_handoff.py tests/test_follow_trace*.py tests/test_wheel_contact_analysis.py > "$base/contracts2.log" 2>&1
result=$?
echo "$result" > "$base/contracts2.exit"
test "$result" = 0 || exit "$result"
python3 -B "$base/probe_admission.py" --output run2 > "$base/run2.log" 2>&1
result=$?
echo "$result" > "$base/run2.exit"
exit "$result"
