"""r98 동결 입력에서3파일과 새 정렬 함수만 교체한 PC/Jetson 묶음."""
import hashlib
import io
import json
from pathlib import Path
import tarfile
import zipfile

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
original=json.loads((A.parent/'control_10hz_cost_r98_v1/bundle_manifest.json').read_bytes())
base=ROOT/'build/r98_pc_frozen';target=ROOT/'build/r103_pc_frozen'
archive=ROOT/'build/jetson_query_set_r103_20261111.tar.gz'
assert not target.exists() and not archive.exists()
before=A/'before.zip'
with zipfile.ZipFile(before) as z:changed=list(z.namelist())
new='src/arena_autonomy/arena_autonomy/point_set.py'
names=list(original['files'])+[new]
inventory={}
target.mkdir()
with tarfile.open(archive,'x:gz',compresslevel=1) as out:
    for name in names:
        if name!=new:
            data=(base/name).read_bytes()
            assert sha(data)==original['files'][name]['sha256']
        if name in changed or name==new:data=(ROOT/name).read_bytes()
        dest=target/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
        inventory[name]=dict(size=len(data),sha256=sha(data))
        item=tarfile.TarInfo(name);item.size=len(data)
        out.addfile(item,io.BytesIO(data))
manifest={**original,'files':inventory,'archive_sha256':sha(archive.read_bytes()),'archive_size':archive.stat().st_size,
    'scope':'r98 identical trace/config and sources except three exact point-set sort callsites plus helper',
    'changed_sources':changed,'new_sources':[new]}
(A/'bundle_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
# observer는 CLI에서 전달한 manifest를 사용하므로 묶음 안 메타데이터도 함께 둔다.
dest=target/'artifacts/validation/2026-11-11/query_set_r103_v1/bundle_manifest.json'
dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes((A/'bundle_manifest.json').read_bytes())
parent='config/tests/vision_completion_r96_sha256.json'
with zipfile.ZipFile(before) as z:
    changes={name:dict(before=sha(z.read(name)),after=sha((ROOT/name).read_bytes()),
        reason='동일 유한 float64 질의점 집합·사전식 순서를 lexsort로 생성; 경계/비유한값은 기존 경로') for name in changed}
contract=dict(schema_version=1,scope=manifest['scope'],parent_contract=parent,
    parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={new:sha((ROOT/new).read_bytes())})
(ROOT/'config/tests/query_set_r103_sha256.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(files=len(inventory),changed=changed,new=new,archive_sha256=manifest['archive_sha256'],bytes=manifest['archive_size'])))
