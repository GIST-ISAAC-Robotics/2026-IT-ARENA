"""r59 원본 보존·계보와 같은 C5 조건을 등록한다. 실제 주행은 별도 실행."""
import hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3];sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/tracked_wall_occluder_r58_sha256.json'
contract='config/tests/forward_rejoin_posts_r59_sha256.json'
snapshot=A/'before_r59.zip';assert not (R/contract).exists()
reasons={
    'src/arena_autonomy/arena_autonomy/bypass_wall_fit.py':'Use physical extent for narrow foreground returns, retaining every obstacle and actual ray continuity',
    'src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py':'Continue an already observed rejoin from current front-body and forward wall support; no initial entry permission or missing-space fill'}
with zipfile.ZipFile(snapshot) as z:
    changes={p:dict(before=sha(z.read(p)),after=sha((R/p).read_bytes()),reason=why) for p,why in reasons.items()}
data=dict(schema_version=1,scope='r59: selected post handling during established stationary-actor rejoin; occupancy, surface, STOP and guard boundaries unchanged',
    parent_contract=parent,parent_contract_sha256=sha((R/parent).read_bytes()),
    before_snapshot=snapshot.relative_to(R).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
(R/contract).write_text(json.dumps(data,indent=2)+'\n',encoding='utf8',newline='\n')
p=R/'tests/test_source_provenance_r0_plus_b3.py';raw=p.read_bytes()
needle=b"M3_SUCCESSORS += ('config/tests/tracked_wall_occluder_r58_sha256.json',)"
assert raw.count(needle)==1
newline=b'\r\n' if b'\r\n' in raw else b'\n'
p.write_bytes(raw.replace(needle,needle+newline+b"M3_SUCCESSORS += ('config/tests/forward_rejoin_posts_r59_sha256.json',)"))
for ext in ('py','sh'):
    out=A/f'c5_run.{ext}';assert not out.exists()
    s=(A/f'c4_run.{ext}').read_text(encoding='utf8').replace('run4','run5').replace('c4_run.py','c5_run.py')
    out.write_text(s,encoding='utf8',newline='\n')
print('r59 and C5 registered; simulation not executed')
