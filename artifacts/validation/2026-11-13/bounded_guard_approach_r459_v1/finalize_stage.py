import difflib
import hashlib
import json
from pathlib import Path
B=Path(__file__).resolve().parent
R=B.parents[3]
original=json.loads((B/'original_sha256.json').read_text())
names=list(original)+['tests/test_bounded_guard_approach.py']
rows=[];patch=[]
for name in names:
    staged=B/'stage'/name
    before=R/name
    old=before.read_bytes() if before.exists() else b''
    new=staged.read_bytes()
    prior=hashlib.sha256(old).hexdigest() if before.exists() else None
    assert prior==original.get(name)
    rows.append(dict(path=name,status='modified' if before.exists() else 'added',original_sha256=prior,
        stage_sha256=hashlib.sha256(new).hexdigest(),stage_path=str(staged.resolve())))
    patch.extend(difflib.unified_diff(old.decode('utf-8').splitlines(keepends=True),
        new.decode('utf-8').splitlines(keepends=True),fromfile='a/'+name if before.exists() else '/dev/null',tofile='b/'+name))
# Minimal readable diff normalizes CRLF only for review; integration uses exact stage SHA.
review=[]
for name in names:
    old=(R/name).read_text(encoding='utf-8') if (R/name).exists() else ''
    new=(B/'stage'/name).read_text(encoding='utf-8')
    review.extend(difflib.unified_diff(old.splitlines(keepends=True),new.splitlines(keepends=True),
        fromfile='a/'+name if (R/name).exists() else '/dev/null',tofile='b/'+name))
(B/'apply.patch').write_text(''.join(patch),encoding='utf-8',newline='')
(B/'review.patch').write_text(''.join(review),encoding='utf-8',newline='')
manifest={str(p.relative_to(B/'stage')).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (B/'stage').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
(B/'changes.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
(B/'stage_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(changed_files=len(rows),stage_files=len(manifest),common_modified=False,changes=rows),ensure_ascii=False))
