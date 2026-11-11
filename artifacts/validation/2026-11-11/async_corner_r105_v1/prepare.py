"""r105 모서리 제안 보완의 변경 전 소스와 계보를 보존한다."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/async_handoff_r104_sha256.json';before=A/'before.zip'
with zipfile.ZipFile(before) as z:
    changes={p:dict(before=sha(z.read(p)),after=sha((ROOT/p).read_bytes()),
        reason='단일 짧은 모서리 사선의 두 직선 제안; 모든 점의 기존 3cm 잔차/각도/길이 조건 유지') for p in z.namelist()}
contract=dict(schema_version=1,scope='r105 observed short corner bevel; async remains opt-in',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
dest=ROOT/'config/tests/short_corner_bevel_r105_sha256.json'
with dest.open('x',encoding='utf8') as f:json.dump(contract,f,ensure_ascii=False,indent=2);f.write('\n')
print(dest)
