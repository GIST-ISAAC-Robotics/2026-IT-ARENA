"""현재 공통 Python 소스를 독립 보존한다. 공통 파일을 수정하지 않는다."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
R = A.parents[3]
manifest = {}
for folder in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'tests'):
    for p in sorted((R/folder).rglob('*.py')):
        if '__pycache__' in p.parts:
            continue
        rel = p.relative_to(R)
        raw = p.read_bytes()
        manifest[rel.as_posix()] = hashlib.sha256(raw).hexdigest()
        for name in ('original', 'stage'):
            target = A/name/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                raise RuntimeError('preserved snapshot already exists: '+str(target))
            target.write_bytes(raw)
(A/'source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps(dict(files=len(manifest), common_modified=False)))
