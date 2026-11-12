"""기존 저속/장착 구조 제한을 통과 상태 검사에도 연결하고 이전 소스를 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
source='src/arena_autonomy/arena_autonomy/released_body_gap.py'
parent='config/tests/released_body_gap_r199_sha256.json'
contract='config/tests/released_mount_gap_r202_sha256.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((A/'sources.json').read_bytes())
assert sha(ROOT/source)==m['before'] and sha(A/'candidate.py')==m['candidate']
with zipfile.ZipFile(A/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as z:z.write(ROOT/source,source)
(ROOT/source).write_bytes((A/'candidate.py').read_bytes())
record=dict(schema_version=1,scope='r202 기존 선언 장착 구조 인접 공백 검사를 기본 OFF RELEASED 후보에 일치. 새로운 위험 상한/실차 승인 아님.',
    parent_contract=parent,parent_contract_sha256=sha(ROOT/parent),
    before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(A/'before_sources.zip'),
    approved_changes={source:dict(before=m['before'],after=sha(ROOT/source),
        reason='같은 structure_aware_v1의 실제/목표0.15m/s·단일 선언 구간·외측 반환/국소5cm 검사를 RELEASED에서도 사용')},
    approved_evaluation_scripts={},new_dependencies={})
with (ROOT/contract).open('x',encoding='utf8') as f:json.dump(record,f,ensure_ascii=False,indent=2);f.write('\n')
p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';text=p.read_text('utf8')
anchor="M3_SUCCESSORS += ('config/tests/released_body_gap_r199_sha256.json',)"
assert text.count(anchor)==1 and contract not in text
p.write_text(text.replace(anchor,anchor+"\nM3_SUCCESSORS += ('"+contract+"',)"),encoding='utf8')
print(contract)
