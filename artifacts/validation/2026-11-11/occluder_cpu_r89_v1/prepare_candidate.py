"""동결r87 완주 입력·관찰기는 유지하고 local_path 계산 구현만 교체한다."""
from pathlib import Path
import copy,hashlib,io,json,tarfile,time

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PARENT=ROOT/'artifacts/validation/2026-11-11/jetson_completed_traffic_r88_v1/bundle_manifest.json'
parent=json.loads(PARENT.read_bytes());manifest=copy.deepcopy(parent)
name='src/arena_autonomy/arena_autonomy/local_path.py'
before=json.loads((A/'before_manifest.json').read_bytes())[name]
assert parent['files'][name]['sha256']==before
sha=lambda data:hashlib.sha256(data).hexdigest()
source=ROOT/'build/jetson_completed_r87_20261111.tar.gz'
assert sha(source.read_bytes())==parent['archive_sha256']
archive=ROOT/'build/jetson_occluder_r89_20261111.tar.gz';local=ROOT/'build/r89_pc_frozen'
assert not archive.exists() and not local.exists()
local.mkdir()
with tarfile.open(source,'r:gz') as inp,tarfile.open(archive,'x:gz',compresslevel=1) as out:
    for member in inp:
        assert member.isfile() and member.name in parent['files']
        data=inp.extractfile(member).read()
        assert sha(data)==parent['files'][member.name]['sha256']
        if member.name==name:data=(ROOT/name).read_bytes()
        dest=(local/member.name).resolve();assert dest.is_relative_to(local.resolve())
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
        item=tarfile.TarInfo(member.name);item.size=len(data);item.mtime=int(time.time())
        out.addfile(item,io.BytesIO(data))
        manifest['files'][member.name]=dict(size=len(data),sha256=sha(data))
changed=[n for n in manifest['files'] if manifest['files'][n]['sha256']!=parent['files'][n]['sha256']]
assert changed==[name]
manifest.update(parent_manifest_sha256=sha(PARENT.read_bytes()),archive_sha256=sha(archive.read_bytes()),
    archive_size=archive.stat().st_size,changed_files=changed,
    scope='fixed r87 completed trace and stage observer; only radial occluder candidate prefilter computation changed')
(A/'candidate_bundle_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(files=len(manifest['files']),changed=changed,archive_size=manifest['archive_size'])))
