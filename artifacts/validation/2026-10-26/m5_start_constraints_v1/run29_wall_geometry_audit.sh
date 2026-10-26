#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run29_wall_geometry_audit
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp scripts/audit_grid_wall_fit_geometry.py "$O/audit_source.py"
python3 -B scripts/audit_grid_wall_fit_geometry.py artifacts/validation/2026-10-26/m5_start_constraints_v1/run28_wall_model_diagnostic/data/summary.json --output "$O/summary.json" > "$O/stdout.txt" 2> "$O/stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
