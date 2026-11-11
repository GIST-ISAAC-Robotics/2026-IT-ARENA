"""낮은 목표도 거절되는 실제 광선 경계와 진행 후보의 직접 위험 반례를 확인한다."""
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import candidate
import arena_autonomy.rear_fringe_trial as rear
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene

A=Path(__file__).resolve().parent
TRACE=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(line) for line in TRACE.open()]
    candidate.install()
    owner,motion=make_controller(events[0]['data'])
    gaps=[];checks=[]
    def profile(frame,event,arg):
        if event!='return' or frame.f_code is not rear._bounded_nonreturn.__code__:
            return
        if not isinstance(arg,dict) or arg.get('reason')!='unbounded_nonreturn':
            return
        f=frame.f_locals;q=f['q'];scene=f['scene'];lo,hi=f['start'],f['end']
        indices=range(max(0,lo-2),min(len(scene.rays),hi+3))
        gaps.append(dict(reason=arg['reason'],group=[lo,hi],query_count=len(q),
            query_bounds=np.vstack([q.min(axis=0),q.max(axis=0)]).tolist(),
            neighbors=[dict(index=i,ray=freeze(asdict(scene.rays[i])),
                            raw_range=freeze(float(f['values'][i])),
                            current_angle_deg=math.degrees(math.atan2(*scene.rays[i].direction_xy[::-1])))
                       for i in indices]))
    for row in events[1:-1]:
        if row['kind']=='reset':owner.reset();continue
        if row['kind']!='call':continue
        raw=row['data']['input']
        if row['seq']==683:
            before=copy.deepcopy(owner)
            _,scan,history=restore_call(raw,motion)
            scene=motion_scene(scan,history,raw['reference_s'],timing_verified=raw['timing_verified'],
                               epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            front=[i for i,r in enumerate(scene.rays) if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<.15]
            assert front
            cases={'unchanged':copy.deepcopy(raw),'near_front':copy.deepcopy(raw),
                   'front_invalid':copy.deepcopy(raw),'high_measured_speed':copy.deepcopy(raw)}
            for i in front:
                cases['near_front']['scan']['measurements']['ranges_m'][i]=.15
                cases['front_invalid']['scan']['measurements']['ranges_m'][i]={'nonfinite':'nan'}
            cases['high_measured_speed']['speed_mps']=.6
            for name,data in cases.items():
                result=freeze(replay_call(copy.deepcopy(before),data,motion))
                n=result[2].get('normal') or {}
                check=n.get('selector_horizon_recheck') or {}
                checks.append(dict(case=name,output=result[:2],reason=result[2].get('reason'),
                                   normal_reason=n.get('reason'),selector=n.get('selector_reason'),
                                   new_retry_applied=bool(check.get('applied'))))
            assert checks[0]['output'][0]==.15
            assert all(c['output'][0]==0 for c in checks[1:])
            break
        try:
            if row['seq']==605:sys.setprofile(profile)
            replay_call(owner,raw,motion)
        finally:
            sys.setprofile(None)
    result=dict(trace_sha256=digest,source_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
                counterexamples=checks,unbounded_groups=gaps,
                scope='Mutated saved sensor inputs at one pre-call state, not physical obstacle or closed-loop driving tests.')
    assert result['source_unchanged'] and checks and gaps
    with (A/'saved_geometry.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='unbounded_groups'}))
    print('unbounded_groups',[(g['group'],len(g['neighbors'])) for g in gaps])


if __name__=='__main__':
    main()
