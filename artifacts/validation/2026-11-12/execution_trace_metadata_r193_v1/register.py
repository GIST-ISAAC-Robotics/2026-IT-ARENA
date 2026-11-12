"""실행 메타데이터 보존 호환 변경. 이전 스냅샷과 해시를 등록한다."""
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
digest = lambda data: hashlib.sha256(data).hexdigest()
parent = 'config/tests/guard_policy_node_r191_sha256.json'
changes, evaluation = {}, {}
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    for name in archive.namelist():
        target = evaluation if name.startswith('scripts/') else changes
        target[name] = dict(before=digest(archive.read(name)), after=digest((ROOT/name).read_bytes()),
            reason='알려진 실행 설정 이벤트를 감사 정보로 보존하고 계산 비용에서 제외. 순번/완료 수/출력 대조 유지.')
result = dict(schema_version=1,
    scope='r193 기록/계측 호환. 실행 지연 재현 또는 제어 정책 변경 아님.',
    parent_contract=parent, parent_contract_sha256=digest((ROOT/parent).read_bytes()),
    before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=digest((A/'before_sources.zip').read_bytes()),
    approved_changes=changes, approved_evaluation_scripts=evaluation, new_dependencies={})
with (ROOT/'config/tests/execution_trace_metadata_r193_sha256.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
