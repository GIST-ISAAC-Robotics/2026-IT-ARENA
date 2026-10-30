#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_preparation_hold_connection_v1
test ! -e "$base/tests1.log"
python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass*.py tests/test_g2*.py tests/test_follow_trace.py tests/test_corridor_handoff_review.py > "$base/tests1.log" 2>&1
python3 -B "$base/check.py" > "$base/run1.log" 2>&1
colcon build --packages-select arena_autonomy > "$base/build1.log" 2>&1
python3 -B - <<'PY' > "$base/installed1.log"
from pathlib import Path
src=Path('src/arena_autonomy/arena_autonomy')
installed=Path('install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy')
files=list(src.glob('*.py'))
assert files and all((installed/p.name).read_bytes()==p.read_bytes() for p in files)
assert Path('install/arena_autonomy/lib/arena_autonomy/g1_g2_handoff_trial').is_file()
print('All',len(files),'autonomy Python files match installed copies; handoff executable exists')
PY
