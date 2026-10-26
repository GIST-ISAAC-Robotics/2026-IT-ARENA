import hashlib,json,zipfile
from pathlib import Path
root=Path.cwd()
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
parent=Path('config/tests/m3_sensor_stress_r0_sha256.json')
snapshot=Path('artifacts/validation/2026-10-25/m3_execution_review_v1/run59_narrow_offset_lead/run/source_snapshot.zip')
with zipfile.ZipFile(snapshot) as z:
    rows={p:dict(before=hashlib.sha256(z.read(p)).hexdigest(),after=digest(Path(p)),reason=reason) for p,reason in {
      'scripts/validate_g1_traffic.py':'M4 명시 시험 옵션·등록 중간 STOP·별도 판정. 기존 교통 판정은 보존',
      'src/arena_bringup/launch/g1_follow.launch.py':'race_control_mode 명시 선택을 전달. 기본 legacy 유지'}.items()}
contract=dict(schema_version=1,scope='M4 시험용 운영 연결의 소스 출처. 기본 채택·실차 안전 승인 아님',
 parent_contract=parent.as_posix(),parent_contract_sha256=digest(parent),before_snapshot=snapshot.as_posix(),before_snapshot_sha256=digest(snapshot),
 approved_changes={p:r for p,r in rows.items() if p.startswith('src/')},approved_evaluation_scripts={p:r for p,r in rows.items() if p.startswith('scripts/')},
 new_dependencies={p:digest(Path(p)) for p in ['scripts/m4_operation_trial.py','scripts/m4_operation_eval.py']})
Path('config/tests/m4_operation_r0_sha256.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')