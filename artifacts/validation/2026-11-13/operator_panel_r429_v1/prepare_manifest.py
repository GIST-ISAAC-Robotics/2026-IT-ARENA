from pathlib import Path
import ast, hashlib, json, difflib
bundle=Path(__file__).resolve().parent
repo=bundle.parents[3]
stage=bundle/'stage'
original=json.loads((bundle/'original_sha256.json').read_text(encoding='utf-8-sig'))
assert all(hashlib.sha256((repo/r['path']).read_bytes()).hexdigest()==r['sha256'] for r in original),'original changed during stage work'
changed=[];patch=[]
for path in sorted(stage.rglob('*.py')):
    ast.parse(path.read_text(encoding='utf-8-sig'))
    rel=path.relative_to(stage).as_posix();dest=repo/rel
    before=dest.read_bytes() if dest.exists() else b'';after=path.read_bytes()
    if before==after:continue
    changed.append(dict(path=rel,original_sha256=hashlib.sha256(before).hexdigest() if dest.exists() else None,
        staged_sha256=hashlib.sha256(after).hexdigest(),bytes=len(after)))
    patch.extend(difflib.unified_diff(before.decode('utf-8-sig').splitlines(True),after.decode('utf-8-sig').splitlines(True),
        fromfile='a/'+rel if dest.exists() else '/dev/null',tofile='b/'+rel))
(bundle/'changes.json').write_text(json.dumps(changed,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(bundle/'changes.patch').write_text(''.join(patch),encoding='utf-8')
print(json.dumps(dict(changed_files=len(changed),originals_unchanged=True,staged_python_parse=True)))
