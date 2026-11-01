"""동일 판단을 재현하며 최초 후반부 무반환 거절의 실제 광선과 외곽을 조사한다."""
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];RUN=HERE/'run1'
sys.path.insert(0,str(ROOT/'scripts'))
from arena_autonomy.follow_trace import replay_events
from arena_autonomy import released_rear_sweep as module

original=module.bounded_released_rear_sweep
samples=[];seen=set()


def probe(scene,raw,q,pad,length,width,speed,target,exterior_limit):
    result=original(scene,raw,q,pad,length,width,speed,target,exterior_limit)
    if result['reason']!='wide_nonreturn' or scene.source_first_s in seen or len(samples)>=8:
        return result
    seen.add(scene.source_first_s)
    rays=scene.rays;status=np.array([r.status for r in rays]);q=np.asarray(q)
    groups=[];i=0
    while i<len(rays):
        if status[i]!='unknown':i+=1;continue
        start=i
        while i+1<len(rays) and status[i+1]=='unknown':i+=1
        end=i;i+=1;a=start-1;b=end+1
        if a<0 or b>=len(rays) or status[a]!='return' or status[b]!='return':continue
        u,v=np.array(rays[a].direction_xy),np.array(rays[b].direction_xy)
        gap=abs(math.atan2(u[0]*v[1]-u[1]*v[0],np.dot(u,v)))
        if gap<=math.radians(3):continue
        groups.append(dict(start=start,end=end,gap_deg=math.degrees(gap),
             positive_inf_only=bool(np.all(np.isposinf(np.asarray(raw.ranges)[start:end+1]))),
             neighbor_ranges_m=[math.dist(rays[k].ray_origin_xy_m,rays[k].hit_xy_m) for k in (a,b)]))
    narrow=np.max(np.abs(q[:,1])-width/2)<=.001
    # 기존 전체 공백 6도 예산을 한 공백에 썼을 때만 미연결로 비교한다.
    trial=module._bounded_nonreturn(scene,raw,q,np.asarray(pad),{**result,'maximum_gap_rad':math.radians(6)}) if narrow else None
    samples.append(dict(reference_s=scene.t_ref_s,source_first_s=scene.source_first_s,
        speed_mps=speed,target_mps=target,original=result,
        query_min_xy=q.min(axis=0).tolist(),query_max_xy=q.max(axis=0).tolist(),
        wide_raw_groups=groups,within_existing_1mm_fringe=bool(narrow),six_degree_diagnostic=trial))
    return result


if __name__=='__main__':
    module.bounded_released_rear_sweep=probe
    source=RUN/'follow_decisions.jsonl';before=sha256(source.read_bytes()).hexdigest()
    try:
        with source.open(encoding='utf-8') as f:replay=replay_events(json.loads(line) for line in f)
    finally:module.bounded_released_rear_sweep=original
    output=dict(baseline={k:v for k,v in replay.items() if k not in ('outcomes','mismatches')},
                unchanged=before==sha256(source.read_bytes()).hexdigest(),input_sha256=before,
                samples=samples,scope='기존 전체 판단은 그대로 재현. 6도 대조는 출력에 연결하지 않은 좁은 기하 진단.')
    with (HERE/'unknown_probe.json').open('x',encoding='utf-8') as f:json.dump(output,f,ensure_ascii=False,indent=2)
    print(json.dumps(output,ensure_ascii=False))
