"""원 동결 archive의 Python 및 native binary를 격리 재사용한다. 빌드 없음."""
from pathlib import Path
import hashlib,json,zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE/'execution_v1'
ROOT.mkdir(exist_ok=False)
source=HERE.parent/'solo_full_lap_r457_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(source/'sources.zip')=='1e966fd9e27eae75dba8b11c56a5018a98c7971569daca39f8c248ec2da24b21'
manifest={}
with zipfile.ZipFile(source/'sources.zip') as archive:
    for name in archive.namelist():
        if name.startswith(('src/arena_autonomy/arena_autonomy/','src/arena_vehicle_interface/arena_vehicle_interface/')):
            data=archive.read(name)
            for mode in ('baseline','candidate'):
                target=ROOT/mode/name
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(data)
patch='src/arena_autonomy/arena_autonomy/local_follow.py'
(ROOT/'candidate'/patch).write_bytes((HERE/'stage'/patch).read_bytes())
for mode in ('baseline','candidate'):
    manifest[mode]={p.relative_to(ROOT/mode).as_posix():sha(p) for p in (ROOT/mode).rglob('*') if p.is_file()}
changes=[n for n,h in manifest['baseline'].items() if manifest['candidate'][n]!=h]
assert changes==[patch]
so=[n for n in manifest['baseline'] if n.endswith('.so')]
assert len(so)==1
assert all(manifest['baseline'][n]==manifest['candidate'][n] for n in so)
cpp=[n for n in manifest['baseline'] if '/_native/' in n]
assert cpp
trace=source/'run1/follow_decisions.jsonl'
trace_sha=sha(trace)
assert trace_sha=='ebaedd424e000dae427d4ae6326bbd202b2bad7e3f32057f008b2ed1a083c0bc'
result={'source_archive_sha256':sha(source/'sources.zip'),'trace_sha256':trace_sha,
        'files':manifest,'source_delta':changes,'native_binary':so,
        'native_source_files':cpp,'build_required':False}
(ROOT/'manifest.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({'files_per_variant':len(manifest['baseline']),'source_delta':changes,
                  'native_binary':{n:manifest['baseline'][n] for n in so},'build_required':False}))
