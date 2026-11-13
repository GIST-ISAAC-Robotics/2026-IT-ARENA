import json,os
from pathlib import Path
A=Path(__file__).resolve().parent.parent
matches=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:
        args=(p/'cmdline').read_bytes().split(b'\0');comm=(p/'comm').read_text().strip()
    except (FileNotFoundError,PermissionError):continue
    if comm.startswith('python') and any(b'solo_full_lap_r457_v1/' in a for a in args):
        matches.append(dict(pid=int(p.name),argv=[a.decode(errors='replace') for a in args if a]))
value=dict(remaining_owned_python_pids=matches,ros_initialized=False,gazebo_started=False,build_executed=False,
           preflight_v2_exitcode=0,resources_returned=True,final_source_freeze=False)
(A/'preflight_v2/cleanup.json').write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(value));assert not matches
