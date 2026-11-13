"""공통 파일을 수정하지 않고 변경 목록/원 SHA/검사용 동결 사본을 만든다."""
from pathlib import Path
import ast
import difflib
import hashlib
import json
import sys
import zipfile

bundle=Path(__file__).resolve().parent;repo=bundle.parents[3];stage=bundle/'stage'
changed=[];patch=[]
for path in sorted(stage.rglob('*.py')):
    ast.parse(path.read_text(encoding='utf-8-sig'))
    rel=path.relative_to(stage).as_posix();dest=repo/rel
    before=dest.read_bytes() if dest.exists() else b'';after=path.read_bytes()
    changed.append(dict(path=rel,original_sha256=hashlib.sha256(before).hexdigest() if dest.exists() else None,
        staged_sha256=hashlib.sha256(after).hexdigest(),bytes=len(after)))
    patch.extend(difflib.unified_diff(before.decode('utf-8-sig').splitlines(True),after.decode('utf-8-sig').splitlines(True),
        fromfile='a/'+rel if dest.exists() else '/dev/null',tofile='b/'+rel))
original=[dict(path=row['path'],sha256=row['original_sha256']) for row in changed]
baseline=bundle/'original_sha256.json'
if baseline.exists():assert json.loads(baseline.read_text())==original,'original source changed'
else:baseline.write_text(json.dumps(original,indent=2),encoding='utf-8')
(bundle/'changes.json').write_text(json.dumps(changed,ensure_ascii=False,indent=2),encoding='utf-8')
(bundle/'changes.patch').write_text(''.join(patch),encoding='utf-8')
files={}
for package in ('arena_autonomy','arena_vehicle_interface'):
    for path in (repo/'src'/package/package).rglob('*.py'):
        rel=path.relative_to(repo).as_posix();selected=stage/rel
        files[package+'/'+path.relative_to(repo/'src'/package/package).as_posix()]=selected if selected.exists() else path
for path in (stage/'src/arena_autonomy/arena_autonomy').rglob('*.py'):
    files['arena_autonomy/'+path.name]=path
files['ros_preview_probe.py']=bundle/'ros_preview_probe.py'
manifest={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in files.items()}
version=sys.argv[1] if len(sys.argv)>1 else 'v1'
with zipfile.ZipFile(bundle/('ros_sources_'+version+'.zip'),'x',zipfile.ZIP_DEFLATED) as z:
    for name,path in files.items():z.write(path,name)
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
print(json.dumps(dict(changed_files=len(changed),frozen_files=len(manifest),python_parse=True)))
