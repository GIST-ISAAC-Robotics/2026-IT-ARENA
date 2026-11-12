"""평가 전용 제한 진단. ROS callback을 막거나 제어/시계에 개입하지 않는다."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import subprocess
import threading
import time


@contextmanager
def observe(output):
    stop=threading.Event()
    began=time.monotonic()
    parent=os.getpid()
    partition=f'arena_autonomy_test_{parent}'
    def collect():
        for deadline in (10.,30.,60.):
            if stop.wait(max(0.,deadline-(time.monotonic()-began))):return
            processes=[]
            for p in Path('/proc').iterdir():
                if not p.name.isdecimal():continue
                try:
                    env=(p/'environ').read_bytes().split(b'\0')
                    if p.name!=str(parent) and ('GZ_PARTITION='+partition).encode() not in env:continue
                    processes.append(dict(pid=int(p.name),command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace'),
                        stat=(p/'stat').read_text(),status=(p/'status').read_text(),wchan=(p/'wchan').read_text()))
                except (OSError,ProcessLookupError):continue
            entry=dict(wall_elapsed_s=time.monotonic()-began,partition=partition,processes=processes,
                memory=Path('/proc/meminfo').read_text())
            command=['gz','topic','-e','-n','1','-t','/world/it_arena_track/stats']
            try:
                result=subprocess.run(command,env={**os.environ,'GZ_PARTITION':partition},capture_output=True,text=True,timeout=4.)
                entry['stats']=dict(returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)
            except subprocess.TimeoutExpired as e:
                entry['stats']=dict(timeout=True,stdout=str(e.stdout or ''),stderr=str(e.stderr or ''))
            (Path(output)/f'environment_{int(deadline)}s.json').write_text(json.dumps(entry,indent=2)+'\n')
    thread=threading.Thread(target=collect,daemon=True,name='evaluation-startup-diagnostic')
    thread.start()
    try:yield
    finally:stop.set();thread.join(timeout=5.)
