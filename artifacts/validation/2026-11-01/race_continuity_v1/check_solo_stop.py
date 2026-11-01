"""단독 멈춤의 실제 입력 꼬리를 10/31 완주 때 이미 있던 조향 구성과 대조한다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE=Path(__file__).resolve().parent
assert __name__=='__main__'
trace=HERE/'solo_1mps/follow_decisions.jsonl'
with (HERE.parents[1]/'2026-10-31/g1_continuous_prefix_course_v1/run1/follow_decisions.jsonl').open() as f:
    past=FollowConfig(**json.loads(next(f))['data']['follow_config'])
keys=('short_boundary_prefix_enabled','parallel_hold_trial','pass_release_trial','guard_curve_trial',
      'traffic_recovery_trial','traffic_path_recovery_trial')
cases={}; observations={}; last_positive=None; fixture_hash=hashlib.sha256()
for line in trace.open('rb'):
    fixture_hash.update(line)
    e=json.loads(line); d=e['data']
    if e['kind']=='header':
        cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
        cases={'as_recorded':LocalFollow(cfg),'previous_short_prefix':LocalFollow(replace(cfg,short_boundary_prefix_enabled=True)),
               'previous_race_follow':LocalFollow(replace(cfg,**{k:getattr(past,k) for k in keys}))}
        observations={k:{'reasons':Counter(),'positive':0,'count':0,'peak':0.,'first_positive_s':None,
                         'changed_targets':0,'changed_angles':0} for k in cases}
    elif e['kind']=='reset':
        for c in cases.values():c.reset()
    elif e['kind']=='call':
        t=d['input']['reference_s']; original,oa,od=d['output']
        if original>0:last_positive=t
        if t<54.:continue
        for k,c in cases.items():
            s,a,b=replay_call(c,d['input'],motion)
            if t<59.62:continue
            z=observations[k];z['count']+=1;z['reasons'][b['reason']]+=1
            z['changed_targets']+=int(abs(s-original)>1e-9);z['changed_angles']+=int(abs(a-oa)>1e-9)
            if s>0:
                z['positive']+=1;z['peak']=max(z['peak'],float(s))
                if z['first_positive_s'] is None:z['first_positive_s']=t
assert observations['as_recorded']['count'] and observations['as_recorded']['changed_targets']==0
report={'scope':'recorded stationary input tail, warmup from 54.0 s; no new trajectory',
        'trace_sha256':fixture_hash.hexdigest(),'last_original_positive_s':last_positive,
        'race_follow_config':{k:getattr(past,k) for k in keys},'choices':observations}
payload=json.dumps(report,indent=2)
with (HERE/'solo_stop_replay.json').open('x') as f:f.write(payload+'\n')
print(payload)
