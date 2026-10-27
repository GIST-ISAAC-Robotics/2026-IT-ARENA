#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests:$PWD/artifacts/validation/2026-10-27/g1_side_hold_preparation_v1${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_side_hold_preparation_v1
test ! -e "$base/run2_tests.log"
/usr/bin/python3 -B - <<'PY'
from pathlib import Path
import zipfile
root=Path.cwd();base=root/'artifacts/validation/2026-10-27/g1_side_hold_preparation_v1'
with zipfile.ZipFile(base/'run2_source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for folder in (root/'src/arena_autonomy/arena_autonomy',base):
        for p in folder.glob('*.py'):z.write(p,p.relative_to(root).as_posix())
PY
/usr/bin/python3 -B -m pytest -q "$base/test_hold_path_candidate.py" > "$base/run2_tests.log" 2>&1
/usr/bin/python3 -B "$base/compare_hold_path.py" > "$base/run2_compare.log" 2>&1
