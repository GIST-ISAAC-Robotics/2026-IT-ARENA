"""실행 전 선택형 변경과 이전 바이트를 등록한다. 원 계약은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sha = lambda data: hashlib.sha256(data).hexdigest()
parent = 'config/tests/depth_batch_r197_sha256.json'
target = 'config/tests/released_body_gap_r199_sha256.json'
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    changes = {name: dict(before=sha(archive.read(name)), after=sha((ROOT/name).read_bytes()),
                         reason='기본 OFF RELEASED 저속 차체 옆 공백 후보와 현재 입력 전체 재검사 연결')
               for name in archive.namelist()}
assert len(changes) == 5 and all(r['before'] != r['after'] for r in changes.values())
dependency = 'src/arena_autonomy/arena_autonomy/released_body_gap.py'
record = dict(schema_version=1, scope='r199 선택형 저속 통과 상태 보완. 새 주행/실차 안전 승인 아님.',
              parent_contract=parent, parent_contract_sha256=sha((ROOT/parent).read_bytes()),
              before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),
              before_snapshot_sha256=sha((A/'before_sources.zip').read_bytes()),
              approved_changes=changes, approved_evaluation_scripts={},
              new_dependencies={dependency: sha((ROOT/dependency).read_bytes())})
with (ROOT/target).open('x', encoding='utf8') as stream:
    json.dump(record, stream, ensure_ascii=False, indent=2); stream.write('\n')
path = ROOT/'tests/test_source_provenance_r0_plus_b3.py'
text = path.read_text('utf8')
anchor = "M3_SUCCESSORS += ('config/tests/depth_batch_r197_sha256.json',)"
assert text.count(anchor) == 1 and target not in text
path.write_text(text.replace(anchor, anchor+"\nM3_SUCCESSORS += ('"+target+"',)"), encoding='utf8')
print(json.dumps(dict(contract=target, changes=len(changes), new_dependencies=1)))
