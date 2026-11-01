"""실제 저장 점군에서 순차/묶음 충돌 검사를 번갈아 측정한다. 실차 마감 시험은 아니다."""
import importlib.util
import json
from pathlib import Path
import time
import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_path import swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('old_sweep', HERE/'reference.py')
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)


def fixtures():
    cases=[]; seconds=set()
    with (HERE.parent/'solo_route_v1/run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line); data=row['data']
            if row['kind']=='header':
                motion=MotionConfig(**data['motion_config']); cfg=data['follow_config']
            if row['kind']!='call':continue
            inp=data['input']; v,steer,detail=data['output']; now=inp['reference_s']
            if v<=0 or int(now) in seconds:continue
            seconds.add(int(now))
            inp,scan,history=restore_call(inp,motion)
            scene=motion_scene(scan,history,now,timing_verified=True,epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
            points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float)
            padding=per_ray_padding(scene,.025+cfg['measurement_padding_m'],cfg['object_motion_bound_mps'])
            horizon=max(2.5,detail.get('observed_search_horizon_m',0.)) if detail.get('open_route_speed_enabled') else 2.5
            for angle in sorted(set((steer,inp['previous_steering_rad']))):
                cases.append((points,dict(steering=angle,wheelbase=cfg['wheelbase_m'],horizon=horizon,margin=padding,length=cfg['length_m'],width=cfg['width_m'])))
    # 첫 표본에서 실제 간섭하는 경우의 회귀 비용도 숨기지 않는다.
    for points,args in list(cases)[::8]:
        cases.append((np.r_[points,[[.05,0.]]],{**args,'margin':np.r_[args['margin'],.055]}))
    return cases


if __name__=='__main__':
    cases=fixtures(); values=[]
    for points,args in cases:
        a=old.swept_limit(points,**args);b=swept_limit(points,**args)
        if a!=b:raise ValueError('수치 결과 불일치')
        values.append(a)
    timings={'reference':[],'batch':[]}; per_case=[{'reference':[],'batch':[]} for _ in cases]
    for rep in range(5):
        order=[('reference',old.swept_limit),('batch',swept_limit)]
        if rep%2:order.reverse()
        for idx,(points,args) in enumerate(cases):
            for name,fn in order:
                started=time.perf_counter_ns(); result=fn(points,**args); elapsed=(time.perf_counter_ns()-started)/1e6
                if result!=values[idx]:raise ValueError('반복에서 결과 변경')
                timings[name].append(elapsed);per_case[idx][name].append(elapsed)
    summary={name:dict(calls=len(ts),total_ms=sum(ts),p50_ms=float(np.percentile(ts,50)),p95_ms=float(np.percentile(ts,95))) for name,ts in timings.items()}
    result=dict(cases=len(cases),matching=True,alternating_repeats=5,summary=summary,
                immediate_cases=[dict(reference_ms=float(np.median(row['reference'])),batch_ms=float(np.median(row['batch']))) for val,row in zip(values,per_case) if val==0.],
                scope='저장 점군과 간섭 대조의 함수 비용. 전체 제어·ROS/Jetson 마감은 아니다.')
    with (HERE/'benchmark.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))
