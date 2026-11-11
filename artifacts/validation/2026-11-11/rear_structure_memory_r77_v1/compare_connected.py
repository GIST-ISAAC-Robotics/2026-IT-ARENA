"""r75 고정 입력을 보존한 변경 전 mixin과 현재 런타임으로 대조한다."""
import hashlib,json,types,zipfile
from pathlib import Path
import arena_autonomy.known_structure as runtime
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
TRACE=HERE.parent/'longitudinal_surface_r75_v1/solo_run1/follow_decisions.jsonl'
OUT=HERE/'connected_compare.json'
if OUT.exists():raise SystemExit('preserve prior comparison')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=[TRACE,ROOT/'src/arena_autonomy/arena_autonomy/known_structure.py',HERE/'before_r77.zip']
hashes={str(p):sha(p) for p in files}
baseline=types.ModuleType('arena_autonomy.known_structure_before_r77')
baseline.__package__='arena_autonomy'
with zipfile.ZipFile(HERE/'before_r77.zip') as z:
    source=z.read('src/arena_autonomy/arena_autonomy/known_structure.py').decode('utf-8')
exec(compile(source,'known_structure_before_r77.py','exec'),baseline.__dict__)
current=runtime.StructureAwareMixin
changed=[];mismatches=[];transfers=[];calls=0
for line in TRACE.open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
        runtime.StructureAwareMixin=baseline.StructureAwareMixin
        before=LocalFollow(cfg)
        runtime.StructureAwareMixin=current
        after=LocalFollow(cfg)
    elif e['kind']=='reset':before.reset();after.reset()
    elif e['kind']=='call':
        a=freeze(replay_call(before,d['input'],motion));b=freeze(replay_call(after,d['input'],motion));calls+=1
        if not equal_output(comparable_decision(a),comparable_decision(d['output'])):mismatches.append(e['seq'])
        detail=b[2].get('partial_observation',{}).get('structure_rear_memory_transfer')
        if detail:transfers.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],evidence=detail))
        if not equal_output(comparable_decision(a),comparable_decision(b)):
            changed.append(dict(seq=e['seq'],time_s=d['input']['reference_s'],before=a[:2],after=b[:2],
                before_reason=a[2].get('reason'),after_reason=b[2].get('reason')))
result=dict(scope='connected runtime, fixed r75 sensor input; no new trajectory',calls=calls,
    baseline_mismatches=mismatches,changed_calls=len(changed),transfer_calls=len(transfers),
    zero_to_positive=sum(a['before'][0]==0 and a['after'][0]>0 for a in changed),
    positive_to_zero=sum(a['before'][0]>0 and a['after'][0]==0 for a in changed),
    input_sha256=hashes,unchanged=all(sha(Path(k))==v for k,v in hashes.items()),changed=changed,transfers=transfers)
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('changed','transfers','input_sha256')},ensure_ascii=False))
raise SystemExit(int(bool(mismatches) or not result['unchanged']))
