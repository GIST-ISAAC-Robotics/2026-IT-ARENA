"""Single owned run and receipt; does not alter frozen r410 sources."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
RECEIPT = A / 'execution_v1.json'
assert not RECEIPT.exists()
assert not (A / 'verify_native_v1').exists()
manifest = A / 'manifest.json'
before = hashlib.sha256(manifest.read_bytes()).hexdigest()
assert before == 'f53ace22f5461e4d2dd66e34303d3323b9f23f411ca1da84851e9e716d051ec8'
command = [sys.executable, '-X', 'utf8', '-B', str(A / 'compare_saved.py'),
           '--output', str(A / 'verify_native_v1')]
started = time.monotonic()
forced = False
with (A / 'execution_v1.log').open('x', encoding='utf8') as log:
    proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    try:
        proc.wait(timeout=1500)
    except subprocess.TimeoutExpired:
        forced = True
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=10)
    try:
        os.killpg(proc.pid, 0)
        residual = True
    except ProcessLookupError:
        residual = False
    if residual:
        forced = True
        os.killpg(proc.pid, signal.SIGTERM)
        time.sleep(1)
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
after = hashlib.sha256(manifest.read_bytes()).hexdigest()
files = json.loads(manifest.read_bytes())['files']
changed = [r['path'] for r in files if hashlib.sha256((ROOT / r['path']).read_bytes()).hexdigest() != r['sha256']]
result = dict(command=command, pid=proc.pid, returncode=proc.returncode,
              elapsed_s=time.monotonic()-started, forced=forced,
              residual_group_after_child_exit=residual, changed_inputs=changed,
              manifest_before=before, manifest_after=after,
              executor='primary GPT; delegated executor reported model capacity before execution')
RECEIPT.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
print(json.dumps(result))
sys.exit(0 if proc.returncode == 0 and not forced and not changed and before == after else 1)
