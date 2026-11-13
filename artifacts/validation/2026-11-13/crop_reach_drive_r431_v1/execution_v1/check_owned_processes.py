import json
from pathlib import Path

output=Path(__file__).resolve().parent
pid=int((output/'run.pid').read_text())
partition=('GZ_PARTITION=arena_autonomy_test_'+str(pid)).encode()
runner=b'artifacts/validation/2026-11-13/crop_reach_drive_r431_v1/run.py'
remaining=[]
errors=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        cmd=(proc/'cmdline').read_bytes().split(b'\0')
        environment=(proc/'environ').read_bytes().split(b'\0')
        if runner in cmd or partition in environment:
            remaining.append(dict(pid=int(proc.name),command=[s.decode(errors='replace') for s in cmd if s]))
    except FileNotFoundError:pass
    except PermissionError:errors.append(int(proc.name))
value=dict(owner_pid=pid,owner_proc_exists=(Path('/proc')/str(pid)).exists(),
           remaining_owned=remaining,permission_denied_pids=errors,
           scope='exact run argument or inherited GZ partition')
with (output/'owned_processes_after.json').open('x') as stream:
    json.dump(value,stream,indent=2);stream.write('\n')
print(json.dumps(value))
