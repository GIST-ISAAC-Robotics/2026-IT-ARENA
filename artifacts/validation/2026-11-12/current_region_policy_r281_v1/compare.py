"""r279의 현재 확인518건 원 재생과 현 저속 공백 후보. 출력 반영/주행 없음."""
import json,sys,time
from pathlib import Path
from collections import Counter
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'opposite_reacquire_runtime_r279_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call,equal_output,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.numeric_kernels import configure
from overlap import evaluate_current,plan_reference
import candidate

with (B/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data']
    calls=[e['data'] for line in f if (e:=json.loads(line))['kind']=='call' and e['data'].get('current_rebase')]
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config']);configure('native_v1')
counts=Counter();transitions=Counter();rows=[]
for c in calls:
    current=c['current_rebase']['input'];submitted=plan_reference(c['input']);expected=c['current_rebase']['output']
    before=time.perf_counter_ns();old=evaluate_current(c['output'],submitted,current,motion,cfg);old_ms=(time.perf_counter_ns()-before)/1e6
    assert equal_output(old,expected)
    d,scan,h=restore_call(current,motion)
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    before=time.perf_counter_ns();new=freeze(candidate.rebase(c['output'],submitted,current,scene,h,cfg,require_current_region=True));new_ms=(time.perf_counter_ns()-before)/1e6
    # old 비용에는 복원/장면 구성도 포함되어 있으므로 이 값끼리 속도 개선을 주장하지 않는다.
    changed=not equal_output(old,new);counts['calls']+=1;counts['different']+=changed
    counts['zero_to_positive']+=old['speed_mps']==0 and new['speed_mps']>0
    counts['positive_to_zero']+=old['speed_mps']>0 and new['speed_mps']==0
    if changed:transitions[old['reason']+' -> '+new['reason']]+=1
    for key in ('reference_s','original_reference_s','source_first_s','path_source_s'):
        assert old.get(key)==new.get(key),key
    if new['speed_mps']>old['speed_mps']:
        assert old['speed_mps']==0 and new['speed_mps']<=.35
        proof=new['current_route_proof'];assert proof['bounded_current_risk']['accepted']
        assert proof['region']['unobserved']>0 and not proof['unknown_filled']
    rows.append(dict(reference_s=d['reference_s'],measured_speed=d['speed_mps'],old_speed=old['speed_mps'],new_speed=new['speed_mps'],old_reason=old['reason'],new_reason=new['reason'],old_full_ms=old_ms,new_core_ms=new_ms))
summary=dict(counts=dict(counts),transitions=dict(transitions),first_changes=[r for r in rows if r['new_speed']!=r['old_speed']][:6],scope='same actual current inputs; no applied output or closed-loop proof; cost scopes differ')
(A/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n');(A/'comparison_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(summary))
