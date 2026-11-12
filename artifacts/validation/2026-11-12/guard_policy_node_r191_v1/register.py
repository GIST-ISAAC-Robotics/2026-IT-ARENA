import hashlib
import json
from pathlib import Path
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
digest=lambda data:hashlib.sha256(data).hexdigest()
parent='config/tests/guard_policy_r190_sha256.json'
changes={}
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    for name in archive.namelist():
        changes[name]=dict(before=digest(archive.read(name)),after=digest((ROOT/name).read_bytes()),
            reason='명시 selected_r181 ROS 설정 전달과 기존 현재 저속 재검사 연결. 기본 원 정책/평가 지연 분리 유지.')
result=dict(schema_version=1,scope='r191 기존 정책 선택 ROS 연결. 기본 정책 불변, 실물/새 주행 승인 아님.',
    parent_contract=parent,parent_contract_sha256=digest((ROOT/parent).read_bytes()),
    before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=digest((A/'before_sources.zip').read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
with (ROOT/'config/tests/guard_policy_node_r191_sha256.json').open('x') as stream:
    json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
