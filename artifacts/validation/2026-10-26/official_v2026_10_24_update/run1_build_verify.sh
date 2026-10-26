#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/official_v2026_10_24_update
mkdir "$O/run1_build_verify"
D="$O/run1_build_verify"
trap 'rc=$?; echo "$rc" > "$D/exit_code.txt"; date -Is > "$D/end.txt"' EXIT
date -Is > "$D/start.txt"
git show HEAD:scripts/build_official_track.py > "$O/builder_before.py"
python3 -B scripts/build_official_track.py > "$D/build.json" 2> "$D/build_stderr.txt"
python3 -B scripts/build_official_track.py --check > "$D/check.json" 2> "$D/check_stderr.txt"
python3 -B "$O/audit_update.py" > "$D/audit_stdout.txt" 2> "$D/audit_stderr.txt"
python3 -B -m pytest -q tests/test_official_track.py > "$D/tests_stdout.txt" 2> "$D/tests_stderr.txt"
gz sdf -k src/arena_gazebo/worlds/it_arena_official/world.sdf > "$D/sdf_stdout.txt" 2> "$D/sdf_stderr.txt"
