"""현재 결측 질의를 직전 실제 반환으로 지지할 수 있는지 저장 입력에서 대조."""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from probe_clearance_geometry import sha, inspect
from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene, transport_points
from arena_autonomy.observation_support import depth_support, transport_scene


def main():
    source=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run3/follow_decisions.jsonl'
    before=sha(source);records=[]
    with source.open(encoding='utf-8') as stream:
        config=MotionConfig(**json.loads(next(stream))['data']['motion_config'])
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];now=data['input']['reference_s']
            if now>134.83:break
            if not any(abs(now-t)<1e-7 for t in (134.72,134.82)):continue
            anchor=data['output'][2]['partial_observation']['anchor_diagnostic']
            inputs,scan,history=restore_call(data['input'],config)
            scene=motion_scene(scan,history,now,timing_verified=inputs['timing_verified'],
                rear_half_angle_deg=0.,epoch=inputs['epoch'])
            records.append((scene,history,np.asarray(thaw(anchor['corrected_points'])),anchor))
    assert len(records)==2
    old,_,old_points,_=records[0]
    current,history,points,anchor=records[1]
    transported=transport_points(old_points,history,old.t_ref_s,current.t_ref_s)
    delta=points-transported
    assert np.allclose(delta,delta[0],rtol=0,atol=1e-10)
    assert np.allclose(delta[0],anchor['fit']['translation_m'],rtol=0,atol=1e-10)
    shift=delta[0]
    previous=transport_scene(old,history,current.t_ref_s)
    def moved(p):return tuple(np.asarray(p)+shift)
    previous=replace(previous,rays=tuple(replace(r,ray_origin_xy_m=moved(r.ray_origin_xy_m),
        hit_xy_m=None if r.hit_xy_m is None else moved(r.hit_xy_m)) for r in previous.rays))
    rows=[]
    for index,point in enumerate(points):
        witness=inspect(current,point)
        queries=np.asarray([w['query'] for w in witness['witnesses']])
        current_support=np.asarray([w['supported'] for w in witness['witnesses']])
        old_pad=np.asarray([.03+2*(current.t_ref_s-r.acquired_at_s) for r in previous.rays])
        old_support=depth_support(previous,queries,old_pad)
        center=depth_support(current,[point],[.03+2*(current.t_ref_s-r.acquired_at_s) for r in current.rays])
        rows.append(dict(index=index,currently_reobserved=anchor['observed'][index],
            current_supported=int(current_support.sum()),previous_supported=int(old_support.sum()),
            union_supported=int(np.logical_or(current_support,old_support).sum()),queries=len(queries),
            current_center_supported=bool(center[0]),
            missing_queries_recovered=int(np.sum(~current_support & old_support)),
            previous_age_s=current.t_ref_s-old.source_first_s))
    result=dict(scope='same approved surface translation; previous real returns retain acquisition ages; diagnostic only',
        current_s=current.t_ref_s,previous_s=old.t_ref_s,translation=shift.tolist(),rows=rows,
        input_sha256=before,input_unchanged=sha(source)==before,script_sha256=sha(Path(__file__)))
    with (BASE/'recent_clearance_probe.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(result,ensure_ascii=True))


if __name__=='__main__':main()
