"""전체 랩 실행 소스 고정. 미커밋 변경도 보존하며 기존 산출물을 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
MANIFEST=OUT/'run1_source_manifest.json'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

if '--check' in sys.argv:
    original=json.loads(MANIFEST.read_text())
    changed=[r['path'] for r in original if not (ROOT/r['path']).is_file() or digest(ROOT/r['path'])!=r['sha256']]
    print(json.dumps(dict(passed=not changed,changed=changed,files=len(original))))
    raise SystemExit(bool(changed))
if MANIFEST.exists():raise FileExistsError(MANIFEST)
paths=sorted(p for folder in ('src','scripts','tests','config') for p in (ROOT/folder).rglob('*')
    if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.pyo'))
MANIFEST.write_text(json.dumps([dict(path=p.relative_to(ROOT).as_posix(),sha256=digest(p)) for p in paths],indent=2))
with zipfile.ZipFile(OUT/'run1_source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for p in paths:archive.write(p,p.relative_to(ROOT).as_posix())
print(json.dumps(dict(passed=True,files=len(paths))))
