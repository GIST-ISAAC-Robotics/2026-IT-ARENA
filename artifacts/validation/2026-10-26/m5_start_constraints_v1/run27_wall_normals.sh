#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run27_wall_normals
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp scripts/parallel_wall_normal_probe.py "$O/normal_probe.py"
cp tests/test_parallel_wall_normal_probe.py "$O/test_source.py"
python3 -B -m pytest -q tests/test_parallel_wall_normal_probe.py tests/test_paired_wall_interval_probe.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/tests_exit_code.txt"
if [ "$result" -eq 0 ]; then
    python3 -B scripts/diagnose_grid_wall_normals.py --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
    result=$?
fi
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
