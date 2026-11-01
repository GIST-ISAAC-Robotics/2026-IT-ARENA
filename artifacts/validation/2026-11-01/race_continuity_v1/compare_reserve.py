"""10/31 입력 그대로, 1.20 m 우회 여유 정책만 바꾼 판단 재생. 차체 궤적은 바뀌지 않는다."""
import hashlib
import json
from collections import Counter
from dataclasses import replace
from pathlib import Path

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


if __name__ == '__main__':
    here = Path(__file__).resolve().parent
    original = here.parents[1]/'2026-10-31/g1_continuous_prefix_course_v1'
    trace = original/'run1/follow_decisions.jsonl'
    review = json.loads((original/'run1_review.json').read_text())
    clock = review['trajectory']
    counts, differing_states = Counter(), Counter()
    motion_changes, mode_times = [], {}
    digest = hashlib.sha256()
    footer = None
    with trace.open('rb') as stream:
        for seq,line in enumerate(stream):
            digest.update(line)
            event=json.loads(line)
            assert event['seq']==seq and footer is None
            d=event['data']
            if event['kind']=='header':
                assert seq==0 and d['schema']==5
                cfg=FollowConfig(**d['follow_config'])
                assert cfg.signed_motion_policy=='legacy_hold'
                motion=MotionConfig(**d['motion_config'])
                execution=replace(HandoffExecutionConfig.from_mapping(d['execution_config']),moving_follow_reserve=True)
                controller=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
            elif event['kind']=='reset':
                controller.reset()
            elif event['kind']=='call':
                t=d['input']['reference_s']
                after,angle,details=replay_call(controller,d['input'],motion)
                before,old_angle,old=d['output']
                counts['all_calls']+=1
                if clock['green_s']<=t<clock['stop_request_s']:
                    counts['race_calls']+=1
                    counts['old_zero']+=before==0
                    counts['new_zero']+=after==0
                    counts['zero_to_positive']+=before==0 and after>0
                    counts['positive_to_zero']+=before>0 and after==0
                    counts['steer_changed']+=abs(angle-old_angle)>1e-9
                    counts['mode_changed']+=old.get('handoff_state')!=details.get('handoff_state')
                    if details['reason'] in ('handoff_entry','handoff_return'):
                        mode_times[details['reason']]=t
                    if before==0 and after>0:
                        differing_states[(old['reason'],details['reason'])]+=1
                        motion_changes.append(after)
            elif event['kind']=='footer':
                footer=d
            else: raise ValueError(event['kind'])
    assert digest.hexdigest()==review['raw_sha256'][trace.name]
    assert footer and all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending')) and footer['written']==seq
    report=dict(scope='recorded 15:1 inputs, same signed policy, moving reserve only; not closed loop',
                trace_sha256=digest.hexdigest(),counts={k:int(v) for k,v in counts.items()},positive_change_range_mps=(
                    [float(min(motion_changes)),float(max(motion_changes))] if motion_changes else None),
                old_new_reasons={'/'.join(k):v for k,v in differing_states.items()},transitions_s=mode_times)
    out=here/'reserve_comparison.json'
    payload=json.dumps(report,indent=2)
    with out.open('x',encoding='utf-8') as f: f.write(payload+'\n')
    print(payload)
