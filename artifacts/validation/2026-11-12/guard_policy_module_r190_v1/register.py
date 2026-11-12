from pathlib import Path
import hashlib
import json
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
digest=lambda value:hashlib.sha256(value).hexdigest()
parent='config/tests/fresh_submission_r184_sha256.json'
changes={}
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    for name in archive.namelist():
        changes[name]=dict(before=digest(archive.read(name)),after=digest((ROOT/name).read_bytes()),
            reason='기존 r181 관측/저속 정책의 인스턴스별 명시 선택 이관. 기본 동작과 판정 조건 불변.')
new='src/arena_autonomy/arena_autonomy/guard_observation_policy.py'
result=dict(schema_version=1,
    scope='r190 기존 시험 정책 공통화. original_v1 기본 유지, selected_r181 명시 선택. 새 주행/실물 승인 아님.',
    parent_contract=parent,parent_contract_sha256=digest((ROOT/parent).read_bytes()),
    before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=digest((A/'before_sources.zip').read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={new:digest((ROOT/new).read_bytes())})
with (ROOT/'config/tests/guard_policy_r190_sha256.json').open('x') as stream:json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
checks={}
for label in ('original_r187','selected_r182','selected_r187'):
    before=json.loads((A/('before_'+label+'.json')).read_text())
    after=json.loads((A/('after_'+label+'.json')).read_text())
    assert before['output_sha256']==after['output_sha256'] and not before['mismatches'] and not after['mismatches']
    checks[label]=dict(calls=before['calls'],sha256=before['output_sha256'],exact=True)
with (A/'equivalence.json').open('x') as stream:json.dump(checks,stream,indent=2);stream.write('\n')
print(json.dumps(checks))
