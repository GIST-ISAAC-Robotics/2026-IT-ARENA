#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - <<'PY'
from pathlib import Path
import json,os,hashlib,datetime,importlib
root=Path.cwd();a=root/'artifacts/validation/2026-11-14/traffic_current_route_r478_v1'
work=[]
for p in Path('/proc').glob('[0-9]*'):
 if int(p.name) in (os.getpid(),os.getppid()):continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0');env=(p/'environ').read_bytes().split(b'\0');exe=(p/'exe').resolve().name
  if exe in ('bash','sh','dash'):continue
  if any(x.startswith(b'GZ_PARTITION=arena_autonomy_test_') for x in env) or exe in ('gzserver','gzclient','gz','ruby') or any(b'validate_g1_traffic' in x or b'traffic_current_route_r478_v1/run.py' in x for x in args[:3]):
   work.append(dict(pid=int(p.name),exe=exe,argv=[x.decode(errors='replace') for x in args if x]))
 except (FileNotFoundError,PermissionError,ProcessLookupError):pass
build=json.loads((root/'artifacts/validation/2026-11-14/solo_full_lap_r475_v1/build_v1/receipt.json').read_bytes())
assert build['build_exitcode']==build['import_cli_exitcode']==0 and build['native_import_verified']
modules={}
for name in ('rclpy','shapely','yaml','arena_autonomy._numeric_kernels','arena_autonomy.race_runtime_policy.current_route_priority','validate_g1_traffic'):
 module=importlib.import_module(name);path=Path(module.__file__)
 modules[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
data=dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),competing_work=work,modules=modules,
    ros_initialized=False,build_reused_from='r475',build_started=False,simulator_started=False)
(a/'execution_v1/environment_preflight.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data));assert not work,work
PY
