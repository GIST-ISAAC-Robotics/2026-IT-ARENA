"""r53 뒤 기본 OFF 갈림길 경로 갱신 변경. 기존 실패·계약을 덮어쓰지 않는다."""
import hashlib
import json
import zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
ROOT=A.parents[3]
target='src/arena_autonomy/arena_autonomy/local_follow.py'
parent='config/tests/signal_posts_marker_launch_r53_sha256.json'
contract='config/tests/junction_priority_r54_sha256.json'
sha=lambda b:hashlib.sha256(b).hexdigest()
snapshot=A/'before_r54.zip'
with zipfile.ZipFile(snapshot) as z:
    before=z.read(target)
after=(ROOT/target).read_bytes()
assert not (ROOT/contract).exists()
record=dict(schema_version=1,scope='r54: default-off junction retains current agreeing opposite wall before short bridge; reject forward divergent side switches; no threshold changes',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes={target:dict(before=sha(before),after=sha(after),reason='Retain visible main path at missing junction markers; reject tangent-only branch switching')},
    approved_evaluation_scripts={},new_dependencies={})
(ROOT/contract).write_text(json.dumps(record,indent=2)+'\n',encoding='utf8',newline='\n')
index=ROOT/'tests/test_source_provenance_r0_plus_b3.py'
s=index.read_text(encoding='utf8')
needle="M3_SUCCESSORS += ('config/tests/signal_posts_marker_launch_r53_sha256.json',)"
assert s.count(needle)==1
s=s.replace(needle,needle+"\nM3_SUCCESSORS += ('config/tests/junction_priority_r54_sha256.json',)")
index.write_text(s,encoding='utf8',newline='\n')
print('r54 registered')
