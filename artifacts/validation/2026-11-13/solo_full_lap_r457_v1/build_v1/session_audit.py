"""같은 WSL 준비 세션의 자신/조상 제외 소유 작업 검사. 서비스 변경 없음."""
import datetime,json,os
from pathlib import Path
A=Path(__file__).resolve().parent.parent
ancestors={os.getpid()};pid=os.getpid()
while pid>1:
    try:pid=int(next(line for line in (Path('/proc')/str(pid)/'status').read_text().splitlines() if line.startswith('PPid:')).split()[1])
    except (FileNotFoundError,StopIteration):break
    ancestors.add(pid)
matches=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in ancestors:continue
    try:args=(p/'cmdline').read_bytes().split(b'\0');comm=(p/'comm').read_text().strip()
    except (FileNotFoundError,PermissionError):continue
    relevant=(any(b'solo_full_lap_r457_v1/' in x for x in args) or
              comm.startswith(('gz-sim','gzserver')) or (args and args[0].endswith(b'ros2') and b'launch' in args))
    if relevant:matches.append(dict(pid=int(p.name),argv=[x.decode(errors='replace') for x in args if x]))
value=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),remaining_owned_or_simulation_work=matches,
           self_and_ancestor_pids_excluded=sorted(ancestors),ros_initialized=False,gazebo_started=False,
           session_exitcode=int(os.environ.get('R457_SESSION_EXIT','0')))
Path(os.environ.get('R457_AUDIT_PATH',str(A/'build_v1/session_cleanup.json'))).write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(value));assert not matches
