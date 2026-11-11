"""r107 초기 제출 위상 보완의 변경 전 소스와 계보."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/short_corner_bevel_r105_sha256.json';before=A/'before.zip'
with zipfile.ZipFile(before) as z:
    changes={p:dict(before=sha(z.read(p)),after=sha((ROOT/p).read_bytes()),
        reason='선택형 별도 판단의 신호/STOP 경계 뒤 다음 새 회전에 첫 제출 정렬; 100ms 판단/양수 적용 간격과 원 시각/STOP 유지') for p in z.namelist()}
contract=dict(schema_version=1,scope='r107 first-source phase alignment; async remains opt-in',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
dest=ROOT/'config/tests/scan_start_phase_r107_sha256.json'
with dest.open('x',encoding='utf8') as f:json.dump(contract,f,ensure_ascii=False,indent=2);f.write('\n')
print(dest)
