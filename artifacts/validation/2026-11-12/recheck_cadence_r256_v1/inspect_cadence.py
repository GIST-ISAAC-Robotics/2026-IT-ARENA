"""저장 입력에서 순차/겹침 실행의 시간 예산과 이전 경로 재사용 조건만 비교한다.

제어기를 다시 실행하거나 이동/입력 시각을 합성하지 않는다. 가정한147/34ms는
r248 p95의 반올림이며 독립적인 실측 최악 조합이나 새 제어기의 출력이 아니다.
"""
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]

def inspect(name):
    trace=ROOT/name
    calls=[]
    with trace.open() as f:
        header=json.loads(next(f))['data']
        for line in f:
            e=json.loads(line)
            if e['kind']=='call':calls.append(e['data'])
    times=[c['input']['reference_s'] for c in calls]
    counts=Counter();examples={};n=0
    for i,data in enumerate(calls):
        normal=data['output'][2].get('normal') or {}
        if data['output'][0]<=0 or normal.get('path_source_s') is None:continue
        # 첫 계산147ms+현재 확인34ms 뒤, 100ms 간격의 다음 확인을 가정한다.
        # 그 시점 이상의 실제 저장 입력만 고르며 미래 자세를 과거로 당기지 않는다.
        target=times[i]+.147+.100
        j=bisect.bisect_left(times,target)
        if j>=len(calls):continue
        old=data['input'];new=calls[j]['input'];origin=normal['path_source_s'];n+=1
        expired=new['reference_s']-origin>=header['follow_config']['memory_ttl_s']
        steer_changed=new['previous_steering_rad']!=old['previous_steering_rad']
        side_changed=new['side']!=old['side']
        epoch_changed=new['epoch']!=old['epoch'] or new['scan']['measurements']['source_epoch']!=old['scan']['measurements']['source_epoch']
        key=('path_expired' if expired else 'path_age_valid')+(' steering_changed' if steer_changed else ' steering_same')
        counts[key]+=1
        counts['side_or_epoch_changed']+=int(side_changed or epoch_changed)
        if key not in examples:
            examples[key]=dict(original_reference_s=times[i],path_source_s=origin,
                requested_current_s=target,actual_stored_current_s=new['reference_s'],
                path_age_s=new['reference_s']-origin,previous_steering_old=old['previous_steering_rad'],
                previous_steering_current=new['previous_steering_rad'],mode=normal.get('selector_mode'))
    return dict(trace=name,sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),calls=len(calls),
        eligible_path_outputs=n,prerequisites=dict(counts),examples=examples,
        limitation='later snapshots belong to original closed loop; not the proposed schedule; no command permission or driving inference')

def main():
    plan=.147;current=.034
    result=dict(assumed_stage_seconds=dict(plan=plan,current=current),
        serial=dict(compute_only_period_s=plan+current,minimum_next_command_age_s=plan+2*current),
        overlapped=dict(compute_only_period_s=max(plan,current),minimum_next_command_age_s=max(plan,current)+current),
        notes=['Periodic 50Hz and new-scan phase waits are omitted from the lower-bound arithmetic.',
               'Overlapping cannot use old stateful outputs without steering/state/epoch validation.',
               'Repeated checking cannot renew path_source_s. A future current-observation proof would be a new policy.'],
        traces=[inspect('artifacts/validation/2026-11-12/'+p+'/two_actor_run1/run1/follow_decisions.jsonl') for p in
                ('recheck_delivery_runtime_r243_v1','staged_event_runtime_r253_v1')])
    (A/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
