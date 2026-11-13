"""종료 후 전용 경로 소스 불변/잔존 프로세스만 읽는다."""
from pathlib import Path
import hashlib
import json
import os

bundle=Path(__file__).resolve().parent
results=[]
for name in ('it-arena-r434-preview-a','it-arena-r434-preview-b'):
    root=Path('/home/jinhyeong')/name
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    changed=[rel for rel,want in manifest.items() if hashlib.sha256((root/rel).read_bytes()).hexdigest()!=want]
    results.append(dict(root=str(root),frozen_files=len(manifest),changed=changed))
owned=[]
for proc in Path('/proc').glob('[0-9]*'):
    if int(proc.name)==os.getpid():continue
    try:cmd=(proc/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace')
    except (OSError,PermissionError):continue
    if 'it-arena-r434-preview-' in cmd or 'ros_preview_probe.py' in cmd:owned.append(dict(pid=int(proc.name),command=cmd))
report=dict(roots=results,owned_processes=owned,passed=not owned and not any(r['changed'] for r in results))
(bundle/'ros_source_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
