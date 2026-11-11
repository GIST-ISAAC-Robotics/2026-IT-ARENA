#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
overlay_pythonpath="${PYTHONPATH:-}"
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${overlay_pythonpath}"
# 동일 소스의 12회귀는 build/r110_regression_recovery.log에서 완료했다.
PYTHONPATH="$overlay_pythonpath" /usr/bin/python3 -B - <<'PY' > build/r110_preflight.log 2>&1
import hashlib, importlib, json
from pathlib import Path
root=Path.cwd();contract=json.loads((root/'config/tests/forward_gap_scope_r109_sha256.json').read_bytes())
for name in ('src/arena_autonomy/arena_autonomy/partial_observation.py','src/arena_autonomy/arena_autonomy/same_path_gap_trial.py'):
    expected=contract['approved_changes'][name]['after']
    module=importlib.import_module('arena_autonomy.'+Path(name).stem)
    loaded=Path(module.__file__)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(root/name)==expected and sha(loaded)==expected,(name,loaded)
    print(name,loaded,expected)
PY
unset ROS_LOCALHOST_ONLY ROS_DOMAIN_ID
python3 -B artifacts/validation/2026-11-11/decision_delay_r110_v1/run.py > build/r110_trial1.log 2>&1
