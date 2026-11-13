from pathlib import Path
import hashlib, json, shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
assert (ROOT/'AGENTS.md').is_file()
for name in ('arena_autonomy', 'arena_vehicle_interface'):
    source = ROOT/'src'/name
    target = A/'stage/src'/name
    if target.exists():
        raise RuntimeError('preserve existing stage')
    shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', 'build', '*.egg-info'))
manifest = {p.relative_to(A/'stage').as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (A/'stage').rglob('*') if p.is_file()}
(A/'before_sha256.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(json.dumps({'files':len(manifest),'root':str(ROOT),'stage':str(A/'stage')}))
