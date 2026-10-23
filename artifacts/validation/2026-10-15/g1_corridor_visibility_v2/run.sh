#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_corridor_visibility_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_corridor_visibility.py tests/test_corner_temporal_shadow.py > "$base/tests.log" 2>&1
python3 -B scripts/audit_corridor_visibility.py artifacts/validation/2026-10-15/g1_solo_lap_v4/run/follow_decisions.jsonl --stateless artifacts/validation/2026-10-15/g1_corridor_shadow_v1/probe/report.json --temporal artifacts/validation/2026-10-15/g1_corridor_temporal_v1/probe/report.json --output "$base/audit" > "$base/audit.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/audit.log"
