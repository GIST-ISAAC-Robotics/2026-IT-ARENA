"""r559 native parser smoke를 한 번 실행하고 소유 child를 회수한다."""
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
STAGE = A.parent / 'r559_optional_launch_parameter_v1/stage/native_parameter_smoke.py'
OUT = A / 'native_smoke_v1'
assert not OUT.exists()
marker = 'r560_' + str(os.getpid())


def owned():
    found = []
    for p in Path('/proc').glob('[0-9]*'):
        try:
            if ('R560_NATIVE_SMOKE_OWNER='+marker).encode() in (p/'environ').read_bytes().split(b'\0'):
                found.append(int(p.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return found


competing = []
for p in Path('/proc').glob('[0-9]*'):
    if int(p.name) in (os.getpid(), os.getppid()):
        continue
    try:
        env = (p/'environ').read_bytes().split(b'\0')
        exe = (p/'exe').resolve().name
        if exe in ('bash','sh','dash'):
            continue
        if any(v.startswith(b'GZ_PARTITION=arena_autonomy_test_') for v in env) or exe in ('gz','gzserver','gzclient','ruby'):
            competing.append(int(p.name))
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        pass
assert not competing, competing
env = dict(os.environ, ROS_DOMAIN_ID='224', ROS_LOCALHOST_ONLY='1',
           ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
           R560_NATIVE_SMOKE_OWNER=marker, PYTHONDONTWRITEBYTECODE='1')
cmd = [sys.executable, '-B', str(STAGE), '--output-dir', str(OUT)]
began = time.monotonic()
process = subprocess.Popen(cmd, cwd=ROOT, env=env, start_new_session=True,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
timed_out = False
forced = False
try:
    output, _ = process.communicate(timeout=40)
except subprocess.TimeoutExpired:
    timed_out = True
    os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        forced = True
        os.killpg(process.pid, signal.SIGKILL)
        output, _ = process.communicate(timeout=5)
remaining = owned()
receipt = dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
               command=cmd, pid=process.pid, exit=process.returncode,
               timed_out=timed_out, forced_cleanup=forced, elapsed_s=time.monotonic()-began,
               competing_before=competing, remaining_owned=remaining, ROS_DOMAIN_ID=224,
               scope='OFF/ON parameter parser only; no Gazebo/control/drive node')
(A/'preflight/native_smoke.log').write_text(output, encoding='utf-8')
(A/'preflight/native_smoke_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
print(json.dumps(receipt))
assert process.returncode == 0 and not timed_out and not remaining
result = json.loads((OUT/'result.json').read_text())
assert result['passed'] and len(result['cases']) == 2
