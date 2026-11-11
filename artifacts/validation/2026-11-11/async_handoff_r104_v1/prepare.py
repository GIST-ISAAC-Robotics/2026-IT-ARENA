"""선택형 연결 변경의 원 바이트/계보. 시뮬레이션은 실행하지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/query_set_r103_sha256.json'
before=A/'before.zip'
with zipfile.ZipFile(before) as z:
    changes={name:dict(before=sha(z.read(name)),after=sha((ROOT/name).read_bytes()),
        reason='기본 inline 유지; 선택형 G1/G2 별도 계산과 원시각/세대/이전 요청각 연결') for name in z.namelist()}
new=['src/arena_autonomy/arena_autonomy/'+name+'.py' for name in ('decision_delivery','decision_process')]
contract=dict(schema_version=1,scope='r104 simulation-only asynchronous handoff; not real-time/hardware approval',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes={k:v for k,v in changes.items() if k.startswith('src/')},
    approved_evaluation_scripts={k:v for k,v in changes.items() if k.startswith('scripts/')},
    new_dependencies={name:sha((ROOT/name).read_bytes()) for name in new})
dest=ROOT/'config/tests/async_handoff_r104_sha256.json'
with dest.open('x',encoding='utf8') as f:json.dump(contract,f,ensure_ascii=False,indent=2);f.write('\n')
print(dest)
