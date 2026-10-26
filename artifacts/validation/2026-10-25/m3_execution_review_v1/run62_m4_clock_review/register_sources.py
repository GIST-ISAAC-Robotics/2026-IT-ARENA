import hashlib,json,zipfile
from pathlib import Path
h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
parent=Path('config/tests/m4_operation_r0_sha256.json')
snapshot=Path('artifacts/validation/2026-10-25/m3_execution_review_v1/run61_b3_same_race/run/source_snapshot.zip')
p='scripts/m4_operation_eval.py'
with zipfile.ZipFile(snapshot) as z:before=hashlib.sha256(z.read(p)).hexdigest()
contract=dict(schema_version=1,scope='M4 CLEAR 경계를 같은 MCU 스트림의 원본 시각/세대로 판정. 기존 주행 실패 보존, 기본 채택 승인 아님',parent_contract=str(parent),parent_contract_sha256=h(parent),before_snapshot=str(snapshot),before_snapshot_sha256=h(snapshot),approved_changes={},approved_evaluation_scripts={p:dict(before=before,after=h(Path(p)),reason='다른 pose 시각과의 1 ms 경계 혼동 제거. CLEAR 뒤 옛 STOP/다른 래치 거절 유지')})
Path('config/tests/m4_operation_clock_r0_sha256.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')