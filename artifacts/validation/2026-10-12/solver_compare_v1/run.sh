#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-12/solver_compare_v1
test ! -e "$base/preflight.log"
python3 -B -m pytest -q tests/test_solver_compare.py tests/test_scheduled_surface_analysis.py tests/test_contact_model_analysis.py > "$base/preflight.log" 2>&1
tail -5 "$base/preflight.log"
python3 -B "$base/register.py"
for action in d1_dantzig p1_pgs; do
    test ! -e "$base/$action.log"
    rc=0
    python3 -B "$base/execute.py" "$action" > "$base/$action.log" 2>&1 || rc=$?
    tail -20 "$base/$action.log"
    if [ "$rc" -ne 0 ]; then exit "$rc"; fi
done
