#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - <<'PY'
import runpy,json,hashlib
from pathlib import Path
a=Path('artifacts/validation/2026-11-14/traffic_current_route_r478_v1')
data=runpy.run_path(str(a/'run.py'),run_name='r478_configure_only')
import validate_g1_traffic as target
data['configure'](target)
value=dict(configure_import_and_geometry=True,ros_initialized=False,simulator_started=False,
    helper_sha256=hashlib.sha256((a/'facility_geometry.py').read_bytes()).hexdigest())
(a/'execution_v1/helper_preflight.json').write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(value))
PY
