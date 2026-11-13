"""WSL 현재 경쟁 실행을 읽는다. ROS/시뮬/장치 실행 없음."""
from pathlib import Path
import json,os
rows=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
    try:
        argv=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
        exe=(p/'exe').resolve().name
        if (exe in ('gz','gzserver','gzclient','ruby','cc1plus','g++','colcon')
            or (exe.startswith('python') and any(v.endswith(('run.py','run_live.py','run_replay.py'))
                or v=='launch' for v in argv[1:]))):rows.append(dict(pid=int(p.name),exe=exe,argv=argv))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
result=dict(remaining=rows,loadavg=Path('/proc/loadavg').read_text().strip(),simulation_started=False)
print(json.dumps(result));assert not rows,rows
