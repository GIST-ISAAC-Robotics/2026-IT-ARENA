"""동결 r82에서 masked_only 계산 하나만 교체한 CPU 비교 묶음."""
import copy
import hashlib
import io
import json
import tarfile
import time
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
parent=ROOT/'artifacts/validation/2026-11-11/path_sampling_r82_v1/jetson_bundle_manifest.json'
manifest=json.loads(parent.read_bytes());before=json.loads((A/'before_manifest.json').read_bytes())
name='src/arena_autonomy/arena_autonomy/m3_execution_trial.py'
assert manifest['files'][name]['sha256']==before[name]
sha=lambda data:hashlib.sha256(data).hexdigest()
source_archive=ROOT/'build/jetson_traffic_r82_20261111.tar.gz'
assert sha(source_archive.read_bytes())==manifest['archive_sha256']
archive=ROOT/'build/jetson_traffic_r85_cpu_20261111.tar.gz'
local=ROOT/'build/r85_pc_frozen'
assert not archive.exists() and not local.exists()
new=copy.deepcopy(manifest);local.mkdir()
with tarfile.open(source_archive,'r:gz') as inp, tarfile.open(archive,'x:gz',compresslevel=1) as out:
    for m in inp:
        assert m.isfile() and m.name in manifest['files'] and (local/m.name).resolve().is_relative_to(local.resolve())
        data=inp.extractfile(m).read();assert sha(data)==manifest['files'][m.name]['sha256']
        if m.name==name:data=(ROOT/name).read_bytes()
        dest=local/m.name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
        item=tarfile.TarInfo(m.name);item.size=len(data);item.mtime=int(time.time())
        out.addfile(item,io.BytesIO(data))
        new['files'][m.name]=dict(size=len(data),sha256=sha(data))
changed=[n for n in new['files'] if new['files'][n]['sha256']!=manifest['files'][n]['sha256']]
assert changed==[name]
new.update(archive_size=archive.stat().st_size,archive_sha256=sha(archive.read_bytes()),
    parent_manifest_sha256=sha(parent.read_bytes()),changed_files=changed,
    scope='fixed r82 inputs/policy, only r85 masked_only computation replaced; not full r84 behavior/ROS performance')
(A/'candidate_bundle_manifest.json').write_text(json.dumps(new,indent=2)+'\n',encoding='utf-8')
measure=(ROOT/'artifacts/validation/2026-11-11/path_sampling_r82_v1/measure_frozen_cost.py').read_text(encoding='utf-8')
measure=measure.replace('frozen r82 traffic offline cost observation only;',
                        'fixed r82 traffic with r85 masked_only computation; offline cost observation only;')
(A/'measure_candidate.py').write_text(measure,encoding='utf-8')
print(json.dumps(dict(files=len(new['files']),changed=changed,archive_size=new['archive_size'])))
