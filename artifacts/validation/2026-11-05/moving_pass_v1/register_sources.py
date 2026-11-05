"""추월 시제품의 소스 사슬을 실제 첫 주행 등록 바이트와 고정한다. 재등록은 허용하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
PARENT='config/tests/continuous_forward_pace_r8_sha256.json'
NAME='config/tests/moving_pass_r9_sha256.json'
REASONS={'src/arena_autonomy/arena_autonomy/bypass_handoff.py':'기존 추종/양보 명령을 유지하고 기본 OFF 추월 선택기를 연결한다.',
    'src/arena_autonomy/arena_autonomy/g1_g2_node.py':'선택형 추월 파라미터와 간결한 상태 보고를 추가한다.',
    'src/arena_bringup/launch/simulation.launch.py':'명시 시뮬레이션 선택·연속 전진율 의존성을 전달한다.',
    'scripts/g1_handoff_eval.py':'선택 실행 인자 및 실제 기록 머리말을 검사한다.',
    'scripts/validate_g1_traffic.py':'추월 선택 CLI를 기존 주행/사후 검사에 전달한다.'}
NEW='src/arena_autonomy/arena_autonomy/moving_pass.py'
def sha(data):return hashlib.sha256(data).hexdigest()
def main():
    before=HERE/'before.zip';after=HERE/'closed_loop_v2/sources.zip';reg=HERE/'closed_loop_v2/registered.json'
    registered=json.loads(reg.read_text(encoding='utf-8'))['sources']
    parent=json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
    old={**parent['approved_changes'],**parent['approved_evaluation_scripts']}
    with zipfile.ZipFile(before) as old_zip,zipfile.ZipFile(after) as new_zip:
        rows={p:dict(before=sha(old_zip.read(p)),after=sha(new_zip.read(p)),reason=r) for p,r in REASONS.items()}
        new_hash=sha(new_zip.read(NEW))
    for p,row in rows.items():
        assert old[p]['after']==row['before'] and row['before']!=row['after']==registered[p],p
    assert registered[NEW]==new_hash
    data=dict(schema_version=1,scope='직선 저속 이동차 추월 첫 후보. 기본 OFF, 실차/고속/기본 운용 승인 아님.',
        parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=before.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(before.read_bytes()),
        after_registration=reg.relative_to(ROOT).as_posix(),after_registration_sha256=sha(reg.read_bytes()),
        approved_changes={k:v for k,v in rows.items() if k.startswith('src/')},
        approved_evaluation_scripts={k:v for k,v in rows.items() if k.startswith('scripts/')},
        new_dependencies={NEW:new_hash})
    with (ROOT/NAME).open('x',encoding='utf-8',newline='\n') as f:json.dump(data,f,indent=2,ensure_ascii=False);f.write('\n')
if __name__=='__main__':main()
