"""정면 단일 결측의 같은 폭/제동 거리 후보. 저장 입력 대조만, 운영 코드 불변."""
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output

ROOT=Path(__file__).resolve().parents[4]
TRACE=Path(__file__).parent/'solo_run1/follow_decisions.jsonl'
OUTPUT=ROOT/'build/c1_native_sampling_2026_11_11/forward_metric_compare.json'
old=PartialObservationPolicy.evaluate
source=inspect.getsource(old)
replacements={
    'metric_gap=widest>math.radians(3)+1e-7':'''single_forward_metric=(self.oblique_metric_enabled and len(runs)==1)
        metric_gap=(widest>math.radians(3)+1e-7 or (single_forward_metric
            and widest>math.radians(1.5)+1e-7
            and np.any(np.abs(angles[unknown])<=math.radians(30)+step/2)))''',
    'minimum_metric_angle = 30. if self.oblique_metric_enabled else 60.':
        'minimum_metric_angle = 0. if single_forward_metric else (30. if self.oblique_metric_enabled else 60.)',
    'if total > math.radians(1.5)+1e-7:':
        'if total > math.radians(1.5)+1e-7 and not (metric_gap and single_forward_metric):',
}
for a,b in replacements.items():
    assert source.count(a)==1
    source=source.replace(a,b)
ns=dict(old.__globals__)
exec(compile(textwrap.dedent(source),'<single-forward-metric-candidate>','exec'),ns)

def run():
    rows=[];mismatch=0
    for line in TRACE.open(encoding='utf-8'):
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            motion=MotionConfig(**d['motion_config']);f=LocalFollow(FollowConfig(**d['follow_config']))
        elif e['kind']=='reset':f.reset()
        elif e['kind']=='call':
            out=replay_call(f,d['input'],motion)
            mismatch+=not equal_output(comparable_decision(freeze(out)),comparable_decision(d['output']))
            rows.append(dict(seq=e['seq'],t=d['input']['reference_s'],measured=d['input']['speed_mps'],
                command=out[:2],reason=out[2]['reason'],observation=out[2].get('partial_observation',{})))
    return rows,mismatch

baseline,mismatches=run()
try:
    PartialObservationPolicy.evaluate=ns['evaluate']
    candidate,_=run()
finally:PartialObservationPolicy.evaluate=old
changes=[dict(seq=a['seq'],time_s=a['t'],measured=a['measured'],before=a['command'],after=b['command'],
    before_reason=a['reason'],after_reason=b['reason']) for a,b in zip(baseline,candidate) if a['command']!=b['command'] or a['reason']!=b['reason']]
summary=dict(scope='saved_input_counterfactual_not_closed_loop',runtime_modified=False,
    calls=len(baseline),baseline_mismatches=mismatches,
    original_zero_to_positive=sum(a['command'][0]==0 and b['command'][0]>0 for a,b in zip(baseline,candidate)),
    positive_to_zero=sum(a['command'][0]>0 and b['command'][0]==0 for a,b in zip(baseline,candidate)),
    trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),candidate_method_sha256=hashlib.sha256(source.encode()).hexdigest(),
    last=candidate[-1])
if OUTPUT.exists():raise FileExistsError(OUTPUT)
OUTPUT.write_text(json.dumps(dict(summary=summary,changes=changes),indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='last'}))
print('last',candidate[-1]['command'],candidate[-1]['reason'])
