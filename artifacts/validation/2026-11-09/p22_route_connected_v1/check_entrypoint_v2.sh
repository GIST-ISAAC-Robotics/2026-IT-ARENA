#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 pkg executables arena_autonomy > artifacts/validation/2026-11-09/p22_route_connected_v1/entrypoints_v2.txt
python3 -B - <<'PY'
from pathlib import Path
p=Path('artifacts/validation/2026-11-09/p22_route_connected_v1/entrypoints_v2.txt')
assert 'arena_autonomy p22_entry_trial' in p.read_text().splitlines()
print('p22_entry_trial executable confirmed')
PY
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - <<'PY'
from arena_autonomy.p22_route_trial import P22Route
from arena_autonomy.p22_entry_trial_node import P22EntryNode
print('P22Route import ok')
print('P22EntryNode import ok')
PY