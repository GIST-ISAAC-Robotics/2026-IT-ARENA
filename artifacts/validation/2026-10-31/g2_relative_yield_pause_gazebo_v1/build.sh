#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_relative_yield_pause_gazebo_v1
test ! -e "$base/build1.log"
colcon build --packages-select arena_autonomy > "$base/build1.log" 2>&1
python3 -B - <<'PYVERIFY' > "$base/installed1.log"
from pathlib import Path
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
src=Path('src/arena_autonomy/arena_autonomy')
installed=Path('install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy')
files=list(src.glob('*.py'))
assert files and all((installed/p.name).read_bytes()==p.read_bytes() for p in files)
assert (installed/'bypass_relative_yield.py').is_file()
assert HandoffExecutionConfig().relative_yield_policy=='off'
print('All',len(files),'autonomy Python files match installed copies; default relative yield OFF')
PYVERIFY
