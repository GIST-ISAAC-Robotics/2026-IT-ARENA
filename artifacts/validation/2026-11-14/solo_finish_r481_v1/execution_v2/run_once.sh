#!/usr/bin/env bash
set -u
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
if ! mkdir native_run_v2; then
  printf '%s\n' 'native_run_v2 already exists; no duplicate batch' >&2
  exit 90
fi
printf '%s\n' "$$" > native_run_v2/owner_bash.pid
date -u +%FT%TZ > native_run_v2/started_utc.txt
python3 -B - <<'PY' > native_run_v2/preflight.json
import hashlib,json,pathlib,sys
a=pathlib.Path.cwd();r=json.loads((a/'prepared.json').read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sys.version_info[:2]==(3,12),sys.version
for p,h in r['helper_files'].items():assert sha(a/p)==h,p
for p,h in r['stage_files'].items():assert sha(a/'stage'/p)==h,p
print(json.dumps(dict(python=sys.version,stage_files=len(r['stage_files']),native_sha256=r['native_sha256'],helpers_verified=True)))
PY
rc=$?
if [ "$rc" -eq 0 ]; then
  python3 -B run_native_tests.py native_run_v2 > native_run_v2/tests.log 2>&1
  test_rc=$?
  printf '%s\n' "$test_rc" > native_run_v2/tests.exit
  python3 -B replay.py candidate native_run_v2 --baseline-result native_run_v1 > native_run_v2/candidate.log 2>&1
  rc=$?
  printf '%s\n' "$rc" > native_run_v2/candidate.exit
  if [ "$rc" -eq 0 ] && [ "$test_rc" -ne 0 ]; then rc=$test_rc; fi
fi
printf '%s\n' "$rc" > native_run_v2/batch.exit
date -u +%FT%TZ > native_run_v2/ended_utc.txt
python3 -B - <<'PY' > native_run_v2/exit_audit.json
import hashlib,json,os,pathlib
a=pathlib.Path.cwd();root=a.parents[3];r=json.loads((a/'prepared.json').read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=json.loads((a/'source_manifest.json').read_bytes())['files']
remaining=[]
for p in pathlib.Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
    try:
        cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except (OSError,ProcessLookupError):continue
    if 'solo_finish_r481_v1' in cmd or 'replay.py baseline native_run_v2' in cmd or 'replay.py candidate native_run_v2' in cmd:
        remaining.append(dict(pid=int(p.name),command=cmd))
result=dict(remaining_owned_processes=remaining,owned_child_count=len(remaining),
    common_unchanged=all(sha(root/p)==h for p,h in old.items()),
    stage_unchanged=all(sha(a/'stage'/p)==h for p,h in r['stage_files'].items()),
    trace_unchanged=sha(a.parent/'solo_variations_r477_v1/cases/start_left20_yaw_plus3/run1/follow_decisions.jsonl')==r['trace_sha256'],
    ros_gazebo_partition_created=False,device_access=False)
print(json.dumps(result,indent=2))
assert not remaining and result['common_unchanged'] and result['stage_unchanged'] and result['trace_unchanged']
PY
audit_rc=$?
printf '%s\n' "$audit_rc" > native_run_v2/exit_audit.exit
if [ "$rc" -eq 0 ] && [ "$audit_rc" -ne 0 ]; then rc=$audit_rc; fi
cat native_run_v2/batch.exit native_run_v2/exit_audit.json
exit "$rc"
