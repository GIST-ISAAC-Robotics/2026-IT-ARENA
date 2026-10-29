#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
BASE="$PWD/artifacts/validation/2026-10-28/bump_dynamic_v1"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B -c 'import rclpy; print(rclpy.__file__)' > "$BASE/env_course_fixed.log" 2>&1
/usr/bin/python3 -B - "$BASE" <<'PY'
from pathlib import Path
import sys,json,hashlib
base=Path(sys.argv[1]).resolve();repo=base.parents[3];out=base/'course1'
meta=json.loads((base/'course1_prepared.json').read_text())
assert all(hashlib.sha256((repo/p).read_bytes()).hexdigest()==h for p,h in meta['source_sha256'].items())
if out.exists():
    assert out.resolve().parent==base and out.is_dir() and not list(out.iterdir())
    (base/'empty_output_before_retry.json').write_text(json.dumps(dict(path=str(out),entries=[],reason='rclpy import failed before launching ROS/Gazebo')))
    out.rmdir()
PY
set +e
/usr/bin/python3 -B "$BASE/run.py" --output "$BASE/course1" --speed-profile brisk --scope course > "$BASE/trial_course2.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/trial_course2.exit"
exit "$code"
