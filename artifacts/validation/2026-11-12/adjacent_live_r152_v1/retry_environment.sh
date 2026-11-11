#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
overlay_pythonpath="${PYTHONPATH:-}"
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${overlay_pythonpath}"
# 지연장치12회귀는 r110에서, 제출창10회귀는 r114 Windows 검사에서 완료했다.
PYTHONPATH="$overlay_pythonpath" /usr/bin/python3 -B - <<'PY' > build/r152_retry_preflight.log 2>&1
import hashlib, importlib, json
from pathlib import Path
root=Path.cwd();contract=json.loads((root/'config/tests/nonreturn_batch_r147_sha256.json').read_bytes())
for name in ('src/arena_autonomy/arena_autonomy/rear_fringe_trial.py','src/arena_autonomy/arena_autonomy/follow_trace.py','src/arena_autonomy/arena_autonomy/partial_observation.py','src/arena_autonomy/arena_autonomy/same_path_gap_trial.py','src/arena_autonomy/arena_autonomy/local_follow.py'):
    expected=contract['approved_changes'][name]['after']
    module=importlib.import_module('arena_autonomy.'+Path(name).stem)
    loaded=Path(module.__file__)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(root/name)==expected and sha(loaded)==expected,(name,loaded)
    print(name,loaded,expected)
PY
unset ROS_LOCALHOST_ONLY ROS_DOMAIN_ID
/usr/bin/python3 -B artifacts/validation/2026-11-12/adjacent_live_r152_v1/run.py --environment-retry > build/r152_trial2.log 2>&1
