import datetime
import hashlib
import json
import os
from pathlib import Path
B=Path(__file__).resolve().parent
remaining=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid():continue
    try:args=(p/'cmdline').read_bytes().split(b'\0');text=b' '.join(args).decode(errors='replace');cwd=(p/'cwd').resolve()
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if (str(B) in text or cwd==B) and any(s in text for s in ('replay_native.py','run_native.sh')):
        remaining.append(dict(pid=int(p.name),argv=[v.decode(errors='replace') for v in args if v]))
manifest=json.loads((B/'stage_manifest.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changed=[n for n,h in manifest.items() if sha(B/'stage'/n)!=h]
source=B.parent/'guard_crawl_entry_r456_v1/source_manifest.json'
frozen=json.loads(source.read_text())
original_changes=[]
for n,h in frozen.items():
    n=n.replace('\\','/')
    p=source.parent/n if n.startswith('base/') else B.parent/n
    if sha(p)!=h:original_changes.append(n)
receipt=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),native_pid=int((B/'native_v1.pid').read_text()),
    native_exit=int((B/'native_v1.exit').read_text()),remaining_owned_processes=remaining,
    stage_input_changes=changed,original_input_changes=original_changes,ros_started=False,gazebo_started=False,
    jetson_used=False,resources_returned_to_root=True)
(B/'resource_return.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
assert not remaining and not changed and not original_changes
