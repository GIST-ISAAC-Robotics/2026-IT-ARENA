"""보존된 r59/r60 실행 바이트로 두 변경의 출처 사슬을 잇는다. 기존 기록은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
F=A.parent/'facility_traffic_stability_v1'
P='src/arena_autonomy/arena_autonomy/pass_release_trial.py'
sha=lambda data:hashlib.sha256(data).hexdigest()


def member(path,name):
    with zipfile.ZipFile(path) as z:return z.read(name)


def main():
    before60=member(F/'before_r60.zip','pass_release_trial.py')
    after60=member(A/'before.zip','pass_release_trial.py')
    after61=(ROOT/P).read_bytes()
    assert before60==subprocess.check_output(['git','show','70dfa9a6:'+P],cwd=ROOT)
    report=json.loads((F/'stationary_r60/run1/report.json').read_bytes())
    assert sha(after60)==report['source_sha256'][P]
    parent='config/tests/forward_rejoin_posts_r59_sha256.json'
    stages=[('current_guard_path_r60',F,before60,after60,
        '기존 GUARD 이동 검사를 현재 G1 경로의 측방 이동 거절에도 적용. 통과 이력·점유·추가 접근 범위 불변.'),
        ('confirmed_passage_fringe_r61',A,after60,after61,
        '두 fresh 통과 확인과 현재 경로에서 기존 1mm 후반부 분류 재사용. 목표0.35 이하·실제 측정 운동·원시 미관측 유지.')]
    for name,folder,before,after,reason in stages:
        snapshot=folder/'contract_before.zip'
        with zipfile.ZipFile(snapshot,'x',zipfile.ZIP_DEFLATED) as z:z.writestr(P,before)
        record=dict(schema_version=1,scope=reason+' 출처 기록이며 실차 안전·기본 채택 승인이 아니다.',
            parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
            before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
            approved_changes={P:dict(before=sha(before),after=sha(after),reason=reason)},
            approved_evaluation_scripts={},new_dependencies={})
        dest=ROOT/'config/tests'/f'{name}_sha256.json'
        with dest.open('x',encoding='utf8') as stream:stream.write(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        parent=dest.relative_to(ROOT).as_posix()
        print(parent,sha(before),sha(after))


if __name__=='__main__':main()
