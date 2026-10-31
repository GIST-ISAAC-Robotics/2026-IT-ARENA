"""동일 저장 입력에 연속 prefix 기하만 대입한다. 물리 궤적은 재생하지 않는다."""
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import arena_autonomy.corridor_observer as observer
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from continuous_prefix import refine

root=Path(__file__).resolve().parent
source=root.parent/'g2_relative_yield_course_video_v1/run1/follow_decisions.jsonl'
before=hashlib.sha256(source.read_bytes()).hexdigest()
out=root/'replay1';out.mkdir(exist_ok=False)
original=observer.short_prefix_evidence
cost=[];used=0;eligible=0


def shadow(points,side,config):
    global used,eligible
    baseline=original(points,side,config)
    if baseline.get('reason')!='insufficient_contiguous_common_prefix':return baseline
    eligible+=1
    start=time.perf_counter()
    candidate=refine(baseline,config)
    cost.append((time.perf_counter()-start)*1000)
    used+=bool(candidate['supported'])
    return candidate


observer.short_prefix_evidence=shadow
changed=[];episodes=[];negative_positive=0;near_bump_changed=0;calls=0
with source.open(encoding='utf-8') as stream,(out/'candidate_calls.jsonl').open('x',encoding='utf-8') as writer:
    for line in stream:
        event=json.loads(line);data=event['data']
        if event['kind']=='header':
            follow=BypassHandoffFollow(FollowConfig(**data['follow_config']),
                HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config'])
        elif event['kind']=='reset':follow.reset()
        elif event['kind']=='call':
            calls+=1
            actual=freeze(replay_call(follow,data['input'],motion))
            now=data['input']['reference_s'];old=data['output']
            row=dict(seq=event['seq'],time_s=now,old=old[:2],new=actual[:2],
                normal_reason=(actual[2].get('normal') or {}).get('reason'),
                selector=(actual[2].get('normal') or {}).get('selector_reason'),
                normal_corridor=(actual[2].get('normal') or {}).get('corridor_candidate'))
            writer.write(json.dumps(row,ensure_ascii=False)+'\n')
            negative_positive+=data['input']['speed_mps']<0 and actual[0]>0
            if not equal_output(old[:2],actual[:2]):
                changed.append(row)
                near_bump_changed+=27.8<=now<=62.4
result=dict(scope='frozen-input command comparison, not closed-loop progress',calls=calls,
    changed_commands=len(changed),first_change_s=changed[0]['time_s'] if changed else None,
    last_change_s=changed[-1]['time_s'] if changed else None,
    hold_to_positive=sum(r['old'][0]==0 and r['new'][0]>0 for r in changed),
    positive_to_hold=sum(r['old'][0]>0 and r['new'][0]==0 for r in changed),
    near_bump_changed=near_bump_changed,negative_feedback_positive_commands=negative_positive,
    candidates_used=used,eligible_calls=eligible,
    refinement_cost_ms=dict(p50=float(np.percentile(cost,50)),p95=float(np.percentile(cost,95)),max=max(cost)) if cost else {},
    input_sha256=before,input_unchanged=before==hashlib.sha256(source.read_bytes()).hexdigest(),
    changed=changed)
(out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='changed'},ensure_ascii=False))
