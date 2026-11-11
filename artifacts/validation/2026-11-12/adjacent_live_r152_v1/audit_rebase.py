"""r126 원 판단과 현재 재계산을 따로 재현한다. 주행 판정은 기존 평가기 소유."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

from rebase import rebase,supported
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import equal_output,freeze,restore_call,replay_call,comparable_decision
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import motion_scene

A=Path(__file__).resolve().parent
assert sys.argv[1:] in ([],['--environment-retry']), 'unknown trial selector'
D=A/('two_actor_run2' if sys.argv[1:] else 'two_actor_run1')


def stats(values):
    if not values:return dict(count=0)
    values=sorted(values)
    return dict(count=len(values),p50=values[len(values)//2],p95=values[int(.95*len(values))],maximum=values[-1])


def main():
    from candidate import install
    install()
    path=D/'run1/follow_decisions.jsonl'
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    events=[json.loads(line) for line in path.open()]
    owner,motion=make_controller(events[0]['data'])
    assert all(event['seq']==i for i,event in enumerate(events))
    footer=events[-1]
    assert footer['kind']=='footer' and sum(row['kind']=='footer' for row in events)==1
    complete=footer['data']
    assert all(complete[k]==0 for k in ('dropped','failed','capture_errors','pending'))
    assert complete['worker_alive'] is False
    assert complete['written']==complete['accepted']==len(events)-1
    planner_calls=0;planner_mismatches=[]
    for row in events[1:-1]:
        if row['kind']=='reset':owner.reset()
        elif row['kind']=='call':
            planner_calls+=1
            actual=freeze(replay_call(owner,row['data']['input'],motion))
            if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):
                planner_mismatches.append(row['seq'])
        elif row['kind']!='delivery_delay_profile':
            raise ValueError('unknown trace event: '+row['kind'])
    reasons=Counter();mismatches=[];count=changed=positive=zeros=0
    gaps=[];ages=[];compute=[];violations=[]
    for row in events:
        if row['kind']!='call' or 'current_rebase' not in row['data']:continue
        raw=row['data'];saved=raw['current_rebase'];count+=1
        data,scan,history=restore_call(saved['input'],motion)
        try:
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            actual=rebase(raw['output'],raw['input'],data,scene,history,owner.config)
        except (MotionUnavailable,ValueError) as error:
            actual=dict(speed_mps=0.,steering_rad=data['previous_steering_rad'],
                reference_s=data['reference_s'],reason='rebase_input_stop',input_error=str(error),
                independent_guard_required=True,runtime_authorized=False)
        if not equal_output(freeze(actual),saved['output']):mismatches.append(row['seq'])
        normal=raw['output'][2].get('normal') or {};out=saved['output']
        reasons[out['reason']]+=1;positive+=out['speed_mps']>0;zeros+=out['speed_mps']<=0
        if (not supported(raw['output'])
                or raw['output'][0]<=0 or out['speed_mps']>raw['output'][0]+1e-10
                or out['reference_s']!=data['reference_s']
                or raw['input']['previous_steering_rad']!=data['previous_steering_rad']):
            violations.append(row['seq'])
        if out['speed_mps']>0:
            age=out['reference_s']-out['path_source_s'];ages.append(age)
            if not 0<=age<owner.config.memory_ttl_s:violations.append(row['seq'])
        changed+=not equal_output([out['speed_mps'],out['steering_rad']],raw['output'][:2])
        gaps.append(out['reference_s']-raw['input']['reference_s'])
        compute.append(saved['computation_wall_ms'])
    result=dict(calls=count,positive=positive,zero=zeros,reasons=dict(reasons),
        original_planner_calls=planner_calls,original_planner_mismatch_sequences=planner_mismatches,
        complete_trace=True,
        command_changed=changed,replay_mismatch_sequences=mismatches,contract_violation_sequences=violations,
        source_age_s=stats(ages),current_recalculation_lag_s=stats(gaps),computation_wall_ms=stats(compute),
        source_sha256=digest,source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest,
        analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Current command recalculation only; original planner and driving verdict remain separate')
    with (D/'rebase_audit.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result))
    return 0 if count and not mismatches and not planner_mismatches and not violations and result['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
