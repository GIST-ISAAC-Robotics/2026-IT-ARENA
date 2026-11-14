import json,os,datetime
from pathlib import Path
a=Path(__file__).resolve().parent.parent;pid=int((a/'execution_v1/run.pid').read_text());partition=('GZ_PARTITION=arena_autonomy_test_'+str(pid)).encode();rows=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0');env=(p/'environ').read_bytes().split(b'\0');text=b' '.join(args).decode(errors='replace')
  own=int(p.name)==pid or partition in env or ('measured_steering_lap_r544_v1' in text and any(w in text for w in ['execute_once.sh','stop_once.sh','run.py','stop_owned.py']))
  if own:rows.append(dict(pid=int(p.name),argv=[x.decode(errors='replace') for x in args if x],partition_match=partition in env))
 except (FileNotFoundError,PermissionError):pass
value=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),owner_pid=pid,partition=partition.decode(),remaining_owned_work=rows)
(a/'execution_v1/owned_processes_after.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value));assert not rows
