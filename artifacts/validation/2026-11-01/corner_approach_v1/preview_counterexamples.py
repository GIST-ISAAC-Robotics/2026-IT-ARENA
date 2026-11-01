"""같은 코너 장면에 전방 차체/무반환을 넣고, 실제 위험까지 속도를 높이려는지 확인."""
from dataclasses import replace
import json
import math
import statistics
import time
from pathlib import Path
import numpy as np
from sensor_turn_probe import classify
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene


def inject(scene,kind):
    rays=[]
    for r in scene.rays:
        ox,oy=r.ray_origin_xy_m;dx,dy=r.direction_xy
        # 진행로 x=0.42 m를 가로질러 선 15 cm 폭 차량의 후면.
        t=(.42-ox)/dx if dx>1e-9 else -1.
        if t>0 and abs(oy+t*dy)<.075:
            if kind=='missing':
                r=replace(r,status='unknown',hit_xy_m=None)
            elif r.status=='return' and t<math.dist(r.ray_origin_xy_m,r.hit_xy_m):
                r=replace(r,hit_xy_m=(.42,oy+t*dy))
        rays.append(r)
    return replace(scene,rays=tuple(rays))


def main():
    base=Path(__file__).resolve().parent;run=base/'solo_2p0'
    samples=json.loads((base/'solo_2p0_sensor_turn_samples.json').read_text())['matched']
    stamp=next(s['reference_s'] for s in samples if 23.<s['reference_s']<24.)
    with (run/'follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data'];cfg=header['follow_config']
        d=next(e['data'] for e in map(json.loads,f) if e['kind']=='call' and e['data']['input']['reference_s']==stamp)
    a,scan,h=restore_call(d['input'],MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,h,a['reference_s'],timing_verified=a['timing_verified'],epoch=a['epoch'],
                       rear_half_angle_deg=a['rear_half_angle_deg'])
    v,angle,o=d['output'];arc=o['swept_clearance_m'];rows={}
    for kind,trial in [('original',scene),('stopped_face',inject(scene,'stopped_face')),('missing',inject(scene,'missing'))]:
        timings=[];reasons=set()
        for _ in range(8):
            t=time.perf_counter()
            reasons.add(classify(trial,np.array(o['path_points']),arc+.20,arc,angle,a['previous_steering_rad'],
                a['speed_mps'],cfg,v,trial.t_ref_s-trial.source_first_s))
            timings.append(1000*(time.perf_counter()-t))
        assert len(reasons)==1
        rows[kind]=dict(result=reasons.pop(),changed_rays=sum(b!=c for b,c in zip(trial.rays,scene.rays)),
                        median_ms=statistics.median(timings),max_ms=max(timings))
    assert rows['original']['result']=='additional_20cm_supported_in_sample'
    assert rows['stopped_face']['changed_rays'] and rows['stopped_face']['result']=='observed_hit_remains'
    assert rows['missing']['changed_rays'] and rows['missing']['result']=='current_ray_support_missing'
    out=dict(reference_s=stamp,rows=rows,scope='원래 경로를 고정한 단일 대표 스캔의 반례. 실차·전체 반응시간이나 모든 각도 가림 증명 아님.')
    with (base/'preview_counterexamples.json').open('x',encoding='utf-8') as f:json.dump(out,f,indent=2,ensure_ascii=False)
    print(json.dumps(out,ensure_ascii=False))


if __name__=='__main__':main()
