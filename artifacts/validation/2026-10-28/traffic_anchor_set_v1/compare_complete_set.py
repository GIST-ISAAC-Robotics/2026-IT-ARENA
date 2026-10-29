"""현재 수정의 전체 저장 입력·독립 보호 대조. 물리 주행이 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(BASE.parent/'traffic_residual_v1'))
from arena_autonomy.follow_trace import freeze,replay_call,equal_output
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from safety_shadow import ShadowSafety

def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

trace=BASE.parent/'traffic_path_recovery_connection_v1/run1_course/follow_decisions.jsonl'
before=digest(trace)
sources={p.relative_to(ROOT).as_posix():digest(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
counts,reasons,anchors=Counter(),Counter(),Counter();events=[];costs=[];late=[]
gate=None
with trace.open() as stream:
    header=json.loads(next(stream))['data']
    cfg=FollowConfig(**header['follow_config']);follow=LocalFollow(cfg);motion=MotionConfig(**header['motion_config'])
    for line in stream:
        row=json.loads(line)
        if row['kind']=='reset':follow.reset();counts['resets']+=1;continue
        if row['kind']!='call':continue
        data=row['data'];t=data['input']['reference_s'];counts['calls']+=1
        start=time.perf_counter();result=replay_call(follow,data['input'],motion)
        costs.append(1000*(time.perf_counter()-start))
        reasons[result[2].get('reason')]+=1
        anchor=(result[2].get('partial_observation') or {}).get('anchor_diagnostic') or {}
        if anchor:anchors[anchor['reason']]+=1
        if gate is None:
            gate=ShadowSafety(RecoveryPolicy(True,True,True),cfg,data['input']['rear_half_angle_deg'])
            assert gate.node.partial_policy is not follow.partial_policy
        safe,status=gate.step(data['input'],motion,result[:2])
        assert 0<=safe[0]<=result[0]+1e-12
        if result[0]==0:assert safe[0]==0;counts['zeros_preserved']+=1
        changed=not equal_output(freeze(result[:2]),data['output'][:2])
        if changed:
            counts['changed_commands']+=1
            events.append(dict(t_s=t,before=data['output'][:2],after=result[:2],safe=safe,
                reason=result[2].get('reason'),selector=result[2].get('selector_reason'),anchor=anchor,
                safety_reason=status.get('reason')))
        if 132<=t<=154:
            counts['late_calls']+=1;counts['late_old_positive']+=int(data['output'][0]>0)
            counts['late_candidate_positive']+=int(result[0]>0);counts['late_safe_positive']+=int(safe[0]>0)
            if anchor.get('released') or len(late)<4 or changed and len(late)<15:
                late.append(dict(t_s=t,command=result[:2],safe=safe,anchor=anchor,
                    partial_reason=(result[2].get('partial_observation') or {}).get('reason'),
                    selector=result[2].get('selector_reason'),safety_reason=status.get('reason')))
report=dict(scope='complete saved-input controller and independent guard function comparison; no new trajectory',
    counts=dict(counts),reasons=dict(reasons),anchor_reasons=dict(anchors),late_samples=late,
    input_sha256=before,input_unchanged=digest(trace)==before,source_sha256=sources,
    source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),
    cost_ms=dict(median=float(np.median(costs)),p99=float(np.percentile(costs,99)),maximum=max(costs)))
with (BASE/'complete_set_comparison.json').open('x') as stream:json.dump(freeze(report),stream,indent=2)
with (BASE/'changed_commands.json').open('x') as stream:json.dump(freeze(events),stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','late_samples')}))
