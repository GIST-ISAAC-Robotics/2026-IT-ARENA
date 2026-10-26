import hashlib,json,zipfile
from pathlib import Path
h=lambda data:hashlib.sha256(data).hexdigest()
root=Path('artifacts/validation/2026-10-25/m3_execution_review_v1')
old=root/'run61_b3_same_race/run/source_snapshot.zip'
parent=Path('config/tests/m4_operation_clock_r0_sha256.json')
p='scripts/m4_operation_eval.py'
s=Path(p).read_text()
s=s.replace("    new_race = operator.get('new_race') is True\n    order = ['new_race' if k == 'policy_stop' else k for k in ORDER] if new_race else ORDER\n",'')
s=s.replace("!= order or len(responses) != len(order)","!= ORDER or len(responses) != len(ORDER)")
s=s.replace("replies['new_race' if new_race else 'policy_stop']","replies['policy_stop']")
s=s.replace("== order,","== ORDER,")
a=s.index('    if new_race:\n'); b=s.index('    result.update(gates=gates',a); s=s[:a]+s[b:]
old_eval=s.encode()
assert h(old_eval)==json.loads(parent.read_text())['approved_evaluation_scripts'][p]['after']
before=root/'run63_m4_new_race_preflight/before_sources.zip'
paths=['scripts/m4_operation_trial.py',p,'scripts/validate_g1_traffic.py']
with zipfile.ZipFile(old) as src,zipfile.ZipFile(before,'w',zipfile.ZIP_DEFLATED) as dst:
    data={path:old_eval if path==p else src.read(path) for path in paths}
    for path,value in data.items():dst.writestr(path,value)
contract=dict(schema_version=1,scope='M4 새 경기에서 이전 신호를 재사용하지 않는 격리 시험. 기존 실패/기본값 보존. 실차 승인 아님',parent_contract=parent.as_posix(),parent_contract_sha256=h(parent.read_bytes()),before_snapshot=before.as_posix(),before_snapshot_sha256=h(before.read_bytes()),approved_changes={},approved_evaluation_scripts={path:dict(before=h(data[path]),after=h(Path(path).read_bytes()),reason='새 경기 명시 시험 순서 및 실제 정지/새 신호 대기 사후 평가. 같은 경기 및 원본 판정 유지') for path in paths})
Path('config/tests/m4_new_race_r0_sha256.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf8')