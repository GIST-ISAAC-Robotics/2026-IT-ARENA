"""이미 끝난 v2와 같은 상태 순서로 긴 추가 정지 구간의 진단만 회수한다."""
import json
from collections import Counter
from compare_trace import BASE,ROOT,preflight,sha,FollowConfig,LocalFollow,MotionConfig,replay_call,freeze,thaw
from perturb import CASES,transform

trace=BASE.parent/'official_traffic_v1/run2_course/follow_decisions.jsonl'
header,_=preflight(trace); before=sha(trace)
output=BASE/'angle_diagnosis_v1'; output.mkdir(exist_ok=False)
source={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
source.update({p.relative_to(ROOT).as_posix():sha(p) for p in (BASE/'perturb.py',BASE/'diagnose_angle.py')})
expected={}
with (BASE/'official_run2_replay_v2/decisions.jsonl').open(encoding='utf-8') as stream:
    for line in stream:
        row=json.loads(line); expected[row['seq']]=row['results']['angle']['command']
follow=LocalFollow(FollowConfig(**header['follow_config'])); motion=MotionConfig(**header['motion_config'])
case=next(c for c in CASES if c.name=='angle'); checked=written=0; reasons=Counter()
with trace.open(encoding='utf-8') as stream,(output/'decisions.jsonl').open('x',encoding='utf-8') as sink:
    for line in stream:
        event=json.loads(line)
        if event['kind']=='reset': follow.reset()
        if event['kind']!='call':continue
        data=event['data']; t=data['input']['reference_s']
        if t>137.:break
        result=replay_call(follow,transform(data['input'],case),motion)
        assert all(abs(a-b)<1e-10 for a,b in zip(result[:2],expected[event['seq']]))
        checked+=1
        if t>=130.:
            sink.write(json.dumps(freeze(dict(seq=event['seq'],t=t,baseline=thaw(data['output']),candidate=result)),allow_nan=False)+'\n')
            written+=1; reasons[result[2].get('reason')]+=1
assert sha(trace)==before and all(sha(ROOT/p)==h for p,h in source.items())
summary=dict(scope='same stateful angle replay; detailed controller output at130..137s; no new drive',
             matching_commands=checked,written=written,reasons=dict(reasons),source_sha256=source,input_sha256=before,
             source_unchanged=True,input_unchanged=True)
(output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}),flush=True)
