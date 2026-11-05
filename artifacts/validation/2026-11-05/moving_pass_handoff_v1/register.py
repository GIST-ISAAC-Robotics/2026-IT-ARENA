"""r15 단독 후보 OFF와 r16 추종 인계를 합친 교통 입력을 실행 전에 보존한다."""
import json
import hashlib
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
BASE=HERE.parent/'moving_pass_withdrawal_v1'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,x):
    with p.open('x',encoding='utf-8') as f:json.dump(x,f,indent=2,ensure_ascii=False);f.write('\n')

previous=read(BASE/'candidate_r14/registered.json')
parent_name='config/tests/solo_straight_budget_r15_sha256.json';parent=read(ROOT/parent_name)
source_name='src/arena_autonomy/arena_autonomy/moving_pass.py'
expected={p:r['after'] for g in ('approved_changes','approved_evaluation_scripts') for p,r in parent[g].items()}
expected.update(parent['new_dependencies'])
for p,h in previous['sources'].items():
    if p==source_name:continue
    assert sha(ROOT/p)==expected.get(p,h),p
paths=set(previous['sources'])|set(expected)|{Path(__file__).relative_to(ROOT).as_posix()}
folder=BASE/'candidate_r16';folder.mkdir(exist_ok=False)
data={**previous,'sources':{p:sha(ROOT/p) for p in sorted(paths)},
      'scope':'r14 같은 가속 상대 10초. r15 단독 속도 후보는 OFF. 복귀 동안 현재 입력으로 G1을 준비해 인계 때의 불필요 0명령을 줄이는 r16.',
      'parent_runtime_contract':parent_name,'parent_runtime_contract_sha256':sha(ROOT/parent_name),
      'extra_expected':'기존 뒤 합류/접촉/노면/STOP 판정 그대로. 복귀→추종 전환의 0명령·차체정지·이후진행을 별도 비교.'}
save(folder/'registered.json',data)
with zipfile.ZipFile(folder/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(paths):z.writestr(p,(ROOT/p).read_bytes())
with zipfile.ZipFile(HERE/'before_r16.zip') as z:before=hashlib.sha256(z.read(source_name)).hexdigest()
assert before==previous['sources'][source_name]!=data['sources'][source_name]
contract=dict(schema_version=1,scope='선택형 이동차 추월 복귀의 G1 인계 연속성. 기본 활성화·실물 승인 아님.',
    parent_contract=parent_name,parent_contract_sha256=sha(ROOT/parent_name),
    before_snapshot=(HERE/'before_r16.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(HERE/'before_r16.zip'),
    after_registration=(folder/'registered.json').relative_to(ROOT).as_posix(),after_registration_sha256=sha(folder/'registered.json'),
    approved_changes={source_name:dict(before=before,after=data['sources'][source_name],
        reason='복귀 중 별도 G1·전방 관측을 현재 입력으로 준비하고 중앙 정렬 후 상태를 넘긴다. 현재 G1 정지·고장·독립 보호는 유지.')},
    approved_evaluation_scripts={},new_dependencies={})
save(ROOT/'config/tests/moving_pass_handoff_r16_sha256.json',contract)
