"""각 주행에 등록한 두 실제 소스와 변경 전 보관본을 대조한다."""
from pathlib import Path
import json
import zipfile
from register_sources import sha,HERE,ROOT

def create(previous,current,before,parent,name,reasons,scope):
    reg=HERE/current/'registered.json'
    old_rows=json.loads((HERE/previous/'registered.json').read_text(encoding='utf-8'))['sources']
    rows=json.loads(reg.read_text(encoding='utf-8'))['sources']
    changed={p for p in rows.keys() & old_rows.keys() if p.startswith(('src/','scripts/'))
             and old_rows[p]!=rows[p]}
    assert changed==set(reasons),changed
    snapshot=HERE/before
    updates={}
    with zipfile.ZipFile(snapshot) as saved,zipfile.ZipFile(HERE/current/'sources.zip') as new:
        for p in sorted(changed):
            assert sha(saved.read(p))==old_rows[p] and sha(new.read(p))==rows[p],p
            updates[p]=dict(before=old_rows[p],after=rows[p],reason=reasons[p])
    data=dict(schema_version=1,scope=scope,parent_contract=parent,
        parent_contract_sha256=sha((ROOT/parent).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
        after_registration=reg.relative_to(ROOT).as_posix(),after_registration_sha256=sha(reg.read_bytes()),
        approved_changes={p:r for p,r in updates.items() if p.startswith('src/')},
        approved_evaluation_scripts={p:r for p,r in updates.items() if p.startswith('scripts/')},new_dependencies={})
    with (ROOT/name).open('x',encoding='utf-8',newline='\n') as f:
        json.dump(data,f,indent=2,ensure_ascii=False);f.write('\n')

if __name__=='__main__':
    base='src/arena_autonomy/arena_autonomy/'
    create('closed_loop_v2','closed_loop_v3','witness_before.zip','config/tests/moving_pass_r9_sha256.json',
        'config/tests/moving_pass_witness_r10_sha256.json',{base+'bypass_entry_trial.py':
        '기존 외벽 선택을 별도 메서드로 분리하고 기본 계산은 그대로 둔다.',base+'moving_pass.py':
        '현재 지지대 가림 양옆의 실제 반환으로 후륜축 경로 기준을 확인한다. 짧은 면의 점유 여유는 바꾸지 않는다.'},
        '직선 저속 이동차 추월 선택형 경로 기준. 기본 운용/실물 승인 아님.')
