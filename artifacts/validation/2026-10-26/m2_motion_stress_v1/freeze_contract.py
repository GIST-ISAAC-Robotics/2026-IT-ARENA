from pathlib import Path
import hashlib
import json

root=Path.cwd(); here=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256((root/p).read_bytes()).hexdigest()
before=json.loads((here/'before_sha256.json').read_text())
parent='config/tests/m3_exact_cost_r0_sha256.json'
contract=dict(schema_version=1,scope='M2 시험용 IMU/단일 모터축 주입·감사 연결. 제어/보호/물성 변경 및 주행 안전 승인 아님',
    parent_contract=parent,parent_contract_sha256=sha(parent),
    before_snapshot=str((here/'before_sources.zip').relative_to(root)).replace('\\','/'),
    before_snapshot_sha256=sha(here/'before_sources.zip'),approved_changes={},approved_evaluation_scripts={},
    new_dependencies={p:sha(p) for p in ['scripts/g1_motion_stress.py']})
for p,digest in before.items():
    contract['approved_evaluation_scripts'][p]=dict(before=digest,after=sha(p),
        reason='기존 기본 무주입 유지. 명시 시험만 단일 모터축 모드·중계 값/시각·독점 발행·프로필 불변을 감사')
(root/'config/tests/m2_motion_stress_r0_sha256.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
