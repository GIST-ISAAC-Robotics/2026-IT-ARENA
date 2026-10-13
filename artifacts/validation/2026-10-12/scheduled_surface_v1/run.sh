#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-12/scheduled_surface_v1
python3 -B "$base/register.py"
for action in s1_official f1_flat; do
    test ! -e "$base/$action.log"
    rc=0
    python3 -B scripts/run_probe_batch.py --plan "$base/plan_$action.json" --status "$base/status_$action.json" > "$base/$action.log" 2>&1 || rc=$?
    tail -20 "$base/$action.log"
    if [ "$rc" -ne 0 ]; then exit "$rc"; fi
done
