"""최초 지속 정지까지 원본 일치 확인 후 후보 재생. 실제 차량 경로는 변하지 않는다."""
import hashlib,json
from pathlib import Path
import numpy as np
from arena_autonomy import local_follow
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from dataclasses import replace

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parents[3]/'artifacts/validation/2026-11-07/solo_straight_newcourse_v1/run1/follow_decisions.jsonl'
events=[json.loads(l) for l in SOURCE.read_text(encoding='utf8').splitlines()]
h=events[0]['data'];cfg=local_follow.FollowConfig(**h['follow_config']);mc=MotionConfig(**h['motion_config'])


def replay(enabled=False):
    ctl=local_follow.LocalFollow(replace(cfg,oblique_gap_trial=enabled));rows=[]
    for event in events[1:]:
        if event['kind']=='reset':ctl.reset()
        elif event['kind']=='call':
            d=event['data'];out=replay_call(ctl,d['input'],mc)
            rows.append(dict(t=d['input']['reference_s'],original=d['output'][:2],new=out[:2],
                reason=out[2].get('reason'),partial=out[2].get('partial_observation'),
                same=bool(np.allclose(out[:2],d['output'][:2],rtol=0,atol=1e-9))))
    return rows


baseline=replay()
assert all(r['same'] for r in baseline),'source replay mismatch; no candidate inference'
after=replay(True)
changed=[r for r in after if not r['same']]
report=dict(scope='saved fixed feedback only, not closed-loop driving or safe adoption',
    source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),baseline_calls=len(baseline),
    baseline_equal=True,changed=len(changed),first_changes=changed[:4],last_changes=changed[-2:],
    changes_before_stall=sum(r['t']<36.92 for r in changed),positive_after_stall=sum(r['t']>=36.92 and r['new'][0]>0 for r in changed),
    changed_reasons=sorted(set(r['reason'] for r in changed)),
    preserved='same 5cm chord, 0.6m/s gate, 0.2m/s cap, 1m flanks, previous-obstacle loss checks; original unknown rays retained',
    candidate_change='explicit FollowConfig oblique_gap_trial, default off')
(HERE/'replay_comparison.json').write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
(HERE/'replay_changed.json').write_text(json.dumps(freeze(changed),ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('first_changes','last_changes')},ensure_ascii=False))
