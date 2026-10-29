"""전 구간 상태를 재생한 뒤 해제 직후의 짧은 재정지만 자세히 기록한다."""
import json
from pathlib import Path
import sys
import time

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),
    str(ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1')]
from mixed_clearance_candidate import MixedRecoveryCandidate
from probe_clearance_geometry import sha
from safety_shadow import ShadowSafety
from arena_autonomy.follow_trace import replay_call,thaw
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def reduced(detail):
    partial=detail.get('partial_observation') or {}
    result={k:detail.get(k) for k in ('reason','behavior','measured_speed_mps','reverse_motion',
        'selector_reason','selector_choice','boundary_candidate_reason','fresh_boundary_accepted',
        'target_speed_before_hold_mps','swept_clearance_m','release_count','corridor_candidate')}
    result['partial']={k:partial.get(k) for k in ('allowed','reason','speed_cap_mps','latch_time_s',
        'latch_reason','latch_evidence','timed_missing_diagnostic')}
    for key in ('anchor_diagnostic','clearance_diagnostic'):
        value=partial.get(key) or {}
        result[key]={k:value[k] for k in ('reason','matched','released','current_recheck_reason','observed',
            'corrected_points','nearest_distances_m','threat_set','mixed_clearance') if k in value}
        fit=value.get('fit') or {}
        result[key]['fit']={k:fit[k] for k in ('accepted','reason','translation_m','small_translation_trial') if k in fit}
        rolling=value.get('rolling_clearance') or {}
        result[key]['rolling']={k:rolling[k] for k in ('reason','proofs','original_reference_s',
            'current_reference_s','consecutive_fresh_sources') if k in rolling}
    return result


def main():
    trace=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run3/follow_decisions.jsonl'
    before=sha(trace);rows=[];calls=0;guard=None
    with trace.open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data'];config=FollowConfig(**header['follow_config'])
        motion=MotionConfig(**header['motion_config']);controller=LocalFollow(config)
        controller.partial_policy=MixedRecoveryCandidate(True,True,True)
        for line in stream:
            event=json.loads(line)
            if event['kind']=='reset':controller.reset();guard=None
            if event['kind']!='call':continue
            inputs=thaw(event['data']['input']);now=inputs['reference_s']
            if now>136.4:break
            if guard is None:guard=ShadowSafety(MixedRecoveryCandidate(True,True,True),config,inputs['rear_half_angle_deg'])
            output=replay_call(controller,inputs,motion)
            safe,status=guard.step(inputs,motion,output[:2]);calls+=1
            if now>=133.9:
                rows.append(dict(seq=event['seq'],t=now,command=output[:2],safe=safe,
                    planner=reduced(output[2]),guard=reduced(status)))
    result=dict(scope='saved-input candidate detail after full prefix; no new driving',calls=calls,
        input_sha256=before,input_unchanged=sha(trace)==before,rows=rows,
        script_sha256=sha(Path(__file__)),candidate_sha256=sha(BASE/'mixed_clearance_candidate.py'))
    with (BASE/'focused_calls.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(dict(calls=calls,focused=len(rows),input_unchanged=result['input_unchanged'])))


if __name__=='__main__':main()
