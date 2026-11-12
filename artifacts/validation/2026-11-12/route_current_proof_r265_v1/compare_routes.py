"""같은 저장 주행의 후속 입력으로 다시 평가한다. 새 폐루프가 아니다."""
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import current_path_recheck
from arena_autonomy.numeric_kernels import configure
import route_proof

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
configure('native_v1')
reports=[]
for name in ('recheck_delivery_runtime_r243_v1','staged_event_runtime_r253_v1'):
    path=A.parent/name/'two_actor_run1/run1/follow_decisions.jsonl'
    with path.open() as f:
        header=json.loads(next(f))['data']
        calls=[e['data'] for e in map(json.loads,f) if e['kind']=='call']
    motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config'])
    refs=[c['input']['reference_s'] for c in calls]
    counts=Counter();reasons=Counter();rows=[]
    for i,d in enumerate(calls):
        if not current_path_recheck.supported(d['output']):continue
        # 원 계획에서 최소200ms 뒤의 실제 저장 입력. 중간 시각의 관측을 만들지 않는다.
        j=bisect.bisect_left(refs,refs[i]+.2-1e-9)
        if j>=len(calls):continue
        current=calls[j]['input'];source=d['output'][2]['normal']['path_source_s']
        age=current['reference_s']-source
        if not cfg.memory_ttl_s<=age<cfg.memory_ttl_s+.1:
            counts['outside_geometry_age_window']+=1;continue
        started=time.perf_counter()
        _,scan,history=restore_call(current,motion)
        scene=motion_scene(scan,history,current['reference_s'],timing_verified=current['timing_verified'],
                           epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
        result=route_proof.rebase(d['output'],d['input'],current,scene,history,cfg)
        elapsed=(time.perf_counter()-started)*1000
        counts['compared']+=1;counts['positive_candidates']+=result['speed_mps']>0
        reasons[result['reason']]+=1
        assert result['path_source_s']==source
        if result['speed_mps']>0:
            assert result['current_route_proof']['accepted']
        rows.append(dict(original_ref=refs[i],current_ref=current['reference_s'],path_age_s=age,
                         speed=result['speed_mps'],reason=result['reason'],cost_ms=elapsed,
                         proof=result.get('current_route_proof')))
    report=dict(trace=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                counts=dict(counts),reasons=dict(reasons),rows=rows)
    reports.append(report)
(A/'result.json').write_text(json.dumps(reports,indent=2)+'\n')
print(json.dumps([{k:v for k,v in r.items() if k!='rows'} for r in reports],indent=2))
