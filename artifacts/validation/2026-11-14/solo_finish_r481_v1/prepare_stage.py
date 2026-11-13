"""r477 동결 archive의 Python/native 모듈만 독립 stage로 보존한다."""
import hashlib,json,zipfile
from pathlib import Path

A=Path(__file__).resolve().parent
BASE=A.parent/'solo_variations_r477_v1'
ROOT=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
prepared=json.loads((BASE/'execution_prepared.json').read_bytes())
assert sha((BASE/'execution_sources.zip').read_bytes())==prepared['source_archive_sha256']
assert not (A/'stage').exists()
records={}
with zipfile.ZipFile(BASE/'execution_sources.zip') as z:
    for name in z.namelist():
        if name.startswith(('src/arena_autonomy/arena_autonomy/','src/arena_vehicle_interface/arena_vehicle_interface/')) and name.endswith(('.py','.so','.pyd')):
            data=z.read(name)
            assert sha(data)==prepared['frozen_inputs'][name]
            p=A/'stage'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
            records[name]=sha(data)
for name in ['src/arena_autonomy/arena_autonomy/local_follow.py','src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py']:
    p=A/'originals'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((A/'stage'/name).read_bytes())
value=dict(archive=str(BASE/'execution_sources.zip'),archive_sha256=prepared['source_archive_sha256'],
    files=records,common_exact_at_start=all((ROOT/p).is_file() and sha((ROOT/p).read_bytes())==h for p,h in records.items()))
(A/'source_manifest.json').write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(files=len(records),common_exact=value['common_exact_at_start'],source='r477 frozen archive')))
