"""운영 미연결: 좁은 부채꼴 반대쪽에 있는 조각의 불가능 교차를 먼저 제외한다.

모든 꼭짓점이 방향 dot<0이면 볼록 조각 전체가 부채꼴 뒤다.
이 사실은 두 기존 cross 반평면의 교집합에서 이미 요구되므로 추가 안전 가정이 아니다.
1e-12 여유는 경계 근처를 기존 clipping으로 보내는 보수적인 계산 선택이다.
"""
import inspect
import json
import math
from pathlib import Path
import time
import zipfile

import numpy as np
from arena_autonomy import observation_support as base
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from test_local_follow import history,scan

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]

def main():
    assert not (OUT/'report.json').exists()
    source=inspect.getsource(base.sweep_queries)
    old='        possible|=~(np.all(low<0,axis=1)|np.all(high>0,axis=1))'
    new=('        forward=np.sum(directions[:,None]*delta,axis=2)\n'
         '        possible|=~(np.all(low<0,axis=1)|np.all(high>0,axis=1)|np.all(forward < -1e-12,axis=1))')
    assert source.count(old)==1
    candidate=source.replace(old,new)
    (OUT/'candidate_function.py').write_text(candidate)
    environment=dict(base.__dict__)
    exec(compile(candidate,'candidate_function.py','exec'),environment)
    fn=environment['sweep_queries']
    source_path=Path(base.__file__)
    before=digest(source_path)
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        z.write(source_path,source_path.relative_to(ROOT))
        z.write(__file__,'probe.py')
    rng=np.random.default_rng(1017)
    cfg=FollowConfig(wheelbase_m=.135)
    rows=[]
    for case,(v,yaw) in enumerate(((0.,0.),(.2,.7),(.9,-1.3),(.3,2.))):
        raw=scan(.5,rng.uniform(.2,12.,500).tolist())
        for index in rng.choice(500,30,replace=False):raw.ranges[index]=math.inf
        scene=motion_scene(raw,history(v,yaw),.635,timing_verified=True,rear_half_angle_deg=30.)
        for angle in (0.,-9.6137665e-9,9.6137665e-9,-.001,.001,-.175,.175,-.37,.37):
            for distance in (.12,.27,.55,.9):
                outputs={};costs={}
                order=[('baseline',base.sweep_queries),('candidate',fn)]
                if len(rows)%2:order.reverse()
                for name,f in order:
                    start=time.perf_counter_ns();outputs[name]=f(scene,angle,distance,cfg)
                    costs[name]=(time.perf_counter_ns()-start)/1e6
                a,an=outputs['baseline'];b,bn=outputs['candidate']
                rows.append(dict(case=case,angle=angle,distance_m=distance,queries=len(a),
                    exact_same=np.array_equal(a,b),thin_same=an==bn,cost_ms=costs))
    result=dict(scope='shadow_query_equivalence_not_runtime_or_ROS_deadline',cases=len(rows),
        all_exact=all(r['exact_same'] and r['thin_same'] for r in rows),rows=rows,
        source_sha256=before,source_unchanged=before==digest(source_path),runtime_changed=False)
    result['timing_ms']={name:dict(p50=float(np.percentile([r['cost_ms'][name] for r in rows],50)),
        p95=float(np.percentile([r['cost_ms'][name] for r in rows],95)),
        maximum=max(r['cost_ms'][name] for r in rows)) for name in ('baseline','candidate')}
    (OUT/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    return 0 if result['all_exact'] and result['source_unchanged'] else 1

if __name__=='__main__':raise SystemExit(main())
