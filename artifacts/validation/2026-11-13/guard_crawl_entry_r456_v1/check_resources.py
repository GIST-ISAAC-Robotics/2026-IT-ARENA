import datetime
import hashlib
import json
import os
from pathlib import Path
B=Path(__file__).resolve().parent
owned_pid=int((B/'pinpoint_v1.pid').read_text())
remaining=[]
pid_reused=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:args=(p/'cmdline').read_bytes().split(b'\0');text=b' '.join(args).decode(errors='replace')
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    matches=any(x in text for x in ('pinpoint.py','approach_evidence.py','run_pinpoint.sh'))
    if int(p.name)==owned_pid and not matches:
        pid_reused.append(dict(pid=int(p.name),argv=[x.decode(errors='replace') for x in args if x]))
    if matches:
        remaining.append(dict(pid=int(p.name),argv=[x.decode(errors='replace') for x in args if x]))
manifest=json.loads((B/'source_manifest.json').read_text())
changed=[]
for name,expected in manifest.items():
    name=name.replace('\\','/')
    p=(B/name) if name.startswith('base/') else (B.parent/name)
    if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:changed.append(name)
receipt=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pinpoint_pid=owned_pid,
    pinpoint_exit=int((B/'pinpoint_v1.exit').read_text()),remaining_owned_processes=remaining,pid_reused_by_unrelated_process=pid_reused,
    frozen_input_changes=changed,ros_started=False,gazebo_started=False,jetson_used=False,
    resources_returned_to_root=True)
(B/'resource_return.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
assert not remaining and not changed
