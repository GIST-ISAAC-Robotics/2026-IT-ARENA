"""실주행 미채택 r61을 보존하고 r60 바이트로 복원한다. 계약으로 되돌림을 기록한다."""
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
P='src/arena_autonomy/arena_autonomy/pass_release_trial.py'
TEST='tests/test_confirmed_passage_fringe.py'
sha=lambda b:hashlib.sha256(b).hexdigest()


def main():
    outcome=json.loads((A/'motion_comparison.json').read_bytes())
    assert outcome['current']['counts'].get('confirmed_accepted',0)==0
    live=(ROOT/P).read_bytes();test=(ROOT/TEST).read_bytes()
    with zipfile.ZipFile(A/'before.zip') as z:old=z.read('pass_release_trial.py')
    parent='config/tests/confirmed_passage_fringe_r61_sha256.json'
    contract=json.loads((ROOT/parent).read_bytes())
    assert sha(live)==contract['approved_changes'][P]['after']
    assert sha(old)==contract['approved_changes'][P]['before']
    snapshot=A/'before_restore_r62.zip'
    with zipfile.ZipFile(snapshot,'x',zipfile.ZIP_DEFLATED) as z:
        z.writestr(P,live);z.writestr(TEST,test)
    preserved=A/'archived_candidate_tests.py'
    with preserved.open('xb') as f:f.write(test)
    assert preserved.read_bytes()==test
    (ROOT/P).write_bytes(old)
    (ROOT/TEST).unlink()
    reason='r61 실제 주행에서0회 채택·코너 미해결. 반복 영역 계산을 남기지 않고 검증된 r60 선택기 바이트로 복원; r61 코드·검사·실패 보존.'
    record=dict(schema_version=1,scope=reason,parent_contract=parent,
        parent_contract_sha256=sha((ROOT/parent).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={P:dict(before=sha(live),after=sha(old),reason=reason)},
        approved_evaluation_scripts={},new_dependencies={})
    with (ROOT/'config/tests/confirmed_fringe_restore_r62_sha256.json').open('x',encoding='utf8') as f:
        f.write(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(restored_sha256=sha((ROOT/P).read_bytes()),snapshot_sha256=sha(snapshot.read_bytes()),
        candidate_tests_preserved=preserved.read_bytes()==test)))


if __name__=='__main__':main()
