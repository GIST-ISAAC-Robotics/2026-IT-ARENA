#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/independent_guard_route_r489_v1/wsl_verify_v1'
test ! -f "$a/started.json"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID=189
export ROS_LOCALHOST_ONLY=1
printf '%s\n' "$$" > "$a/shell.pid"
python3 -B - "$a" <<'PY'
import hashlib,json,os,sys
from pathlib import Path
a=Path(sys.argv[1]);root=Path.cwd()
receipt=json.loads((a.parent/'integration_v1/applied.json').read_bytes())
sources={r['path'].replace('\\','/'):hashlib.sha256((root/r['path'].replace('\\','/')).read_bytes()).hexdigest() for r in receipt['files']}
assert all(sources[r['path'].replace('\\','/')]==r['common_after_sha256'] for r in receipt['files'])
import arena_autonomy.lidar_safety as guard
assert Path(guard.__file__).resolve()==(root/'src/arena_autonomy/arena_autonomy/lidar_safety.py').resolve()
(a/'started.json').write_text(json.dumps(dict(source_hashes=sources,build_required=False,
    reason='Python source import with existing Jazzy/installed message dependencies',source_import=guard.__file__),indent=2))
PY
set +e
timeout --kill-after=5s 180s python3 -B - "$a" > "$a/pytest.log" 2>&1 <<'PY'
import os,sys
from pathlib import Path
import pytest
a=Path(sys.argv[1]);(a/'pytest.pid').write_text(str(os.getpid()))
raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','--junitxml='+str(a/'pytest.xml'),
    'tests/test_independent_guard_route.py','tests/test_lidar_safety.py',
    'tests/test_partial_observation.py','tests/test_known_structure_policy.py',
    'tests/test_traffic_recovery_contract.py','tests/test_runtime_installed_limits.py']))
PY
test_rc=$?
printf '%s\n' "$test_rc" > "$a/pytest.exit"
timeout --kill-after=5s 30s python3 -B "$a/constructor.py" > "$a/constructor.log" 2>&1
constructor_rc=$?
printf '%s\n' "$constructor_rc" > "$a/constructor.exit"
set -e
python3 -B - "$a" <<'PY'
import hashlib,json,os,sys
from pathlib import Path
a=Path(sys.argv[1]);before=json.loads((a/'started.json').read_bytes())['source_hashes']
unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
alive=[]
for name in ('pytest.pid','constructor.pid'):
    p=a/name
    if p.exists():
        pid=int(p.read_text())
        if Path('/proc',str(pid)).exists():alive.append(pid)
result=dict(source_unchanged=unchanged,owned_child_pids_remaining=alive,
    pytest_exit=int((a/'pytest.exit').read_text()),constructor_exit=int((a/'constructor.exit').read_text()))
(a/'collected.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
assert unchanged and not alive
PY
exit 0
