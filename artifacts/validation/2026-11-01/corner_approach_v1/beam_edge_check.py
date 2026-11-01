"""선택한 동일 스캔에서 진행 방향의 표적·무반환과 뒤쪽 무반환을 구분한다."""
from dataclasses import replace
import json
import math
from pathlib import Path
from statistics import median
from time import perf_counter
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from preview_counterexamples import inject
from ray_turn_preview import classify


BASE=Path(__file__).resolve().parent

def main():
    original=json.loads((BASE/'preview_counterexamples.json').read_text(encoding='utf-8'))
    stamp=original['reference_s']
    with (BASE/'solo_2p0/follow_decisions.jsonl').open() as stream:
        header=json.loads(next(stream))['data']; cfg=header['follow_config']
        d=next(x['data'] for x in map(json.loads,stream) if x['kind']=='call' and x['data']['input']['reference_s']==stamp)
    a,scan,h=restore_call(d['input'],MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,h,a['reference_s'],timing_verified=a['timing_verified'],
                       epoch=a['epoch'],rear_half_angle_deg=a['rear_half_angle_deg'])
    target,angle,o=d['output'];arc=o['swept_clearance_m']; cases={'original':scene}
    for k in ('stopped_face','missing'):
        cases[k]=inject(scene,k)
    cases['rear_only_missing']=replace(scene,rays=tuple(
        replace(r,status='unknown',hit_xy_m=None) if r.status=='return' and
        abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(120) else r for r in scene.rays))
    rows={}
    for k,s in cases.items():
        costs=[]; labels=set()
        for _ in range(10):
            start=perf_counter()
            labels.add(classify(s,np.asarray(o['path_points']),arc+.20,arc,angle,
                                a['previous_steering_rad'],a['speed_mps'],cfg,target,scene.t_ref_s-scene.source_first_s))
            costs.append((perf_counter()-start)*1000)
        assert len(labels)==1
        rows[k]=dict(label=labels.pop(),ms_median=median(costs),ms_max=max(costs),
                     changed_rays=sum(b!=c for b,c in zip(s.rays,scene.rays)))
    assert rows['stopped_face']['label']=='observed_hit_remains'
    assert rows['missing']['label']=='current_ray_support_missing'
    assert rows['original']['label']==rows['rear_only_missing']['label']=='additional_20cm_supported_in_sample'
    assert rows['rear_only_missing']['changed_rays']>0
    with (BASE/'preview_beam_edges.json').open('x',encoding='utf-8') as f:
        json.dump(dict(sim_time_s=stamp,rows=rows,scope='동일 저장 스캔 한 장의 PC 비용과 센서 반례. 노드·Jetson 전체 지연 결과가 아니다.'),f,ensure_ascii=False,indent=2)
    print(rows)


if __name__=='__main__':main()
