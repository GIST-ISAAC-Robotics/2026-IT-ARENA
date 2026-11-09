"""관측 창 변경의 이전 바이트·후속 계보를 보존한다. 주행 통과 선언 아님."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
target='src/arena_autonomy/arena_autonomy/nearby_face_motion.py'
parent='config/tests/nearby_face_motion_r56_sha256.json'
contract='config/tests/nearby_long_window_r57_sha256.json'
snapshot=A/'before_r57.zip';sha=lambda b:hashlib.sha256(b).hexdigest()
with zipfile.ZipFile(snapshot) as z:before=z.read(target)
assert not (R/contract).exists()
data=dict(schema_version=1,scope='r57: nearby faces require 9 distinct scans over .8 seconds, same window as rear face; unchanged approach/noise/onset assumptions, default execution scope unchanged',
    parent_contract=parent,parent_contract_sha256=sha((R/parent).read_bytes()),
    before_snapshot=snapshot.relative_to(R).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes={target:dict(before=sha(before),after=sha((R/target).read_bytes()),
        reason='Use longer measured face history for endpoint error per time, preserving current approach and loss rejection')},
    approved_evaluation_scripts={},new_dependencies={})
(R/contract).write_text(json.dumps(data,indent=2)+'\n',encoding='utf8',newline='\n')
p=R/'tests/test_source_provenance_r0_plus_b3.py';raw=p.read_bytes()
needle=b"M3_SUCCESSORS += ('config/tests/nearby_face_motion_r56_sha256.json',)"
assert raw.count(needle)==1
newline=b'\r\n' if b'\r\n' in raw else b'\n'
p.write_bytes(raw.replace(needle,needle+newline+b"M3_SUCCESSORS += ('config/tests/nearby_long_window_r57_sha256.json',)"))
print('r57 source registered')
