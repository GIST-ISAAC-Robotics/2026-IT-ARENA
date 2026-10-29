"""정지 초기 자세의 독립 거리 잡음 비교. 시간 연속 주행/실물 정확도 시험 아님."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryGuide,EntryConfig,_compare_entries
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from probe_bypass_entry import sensor_scene

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
OUTPUT=BASE/'initial_range_noise_v1.json'
if OUTPUT.exists():raise ValueError('preserve previous comparison')
paths=[ROOT/'src/arena_autonomy/arena_autonomy/bypass_entry_trial.py',Path(__file__)]
original={str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
for t in np.arange(.2,.801,.01):history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
rows=[]
for y in (-.15,0.,.15):
    for noise,seed in [(0.,17)]+[(n,s) for n in (.01,.03) for s in (17,23,42)]:
        rng=np.random.default_rng(seed)
        entry=EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.27,.27))
        guide=EntryGuide(entry=entry,stationary_outer_boundary=True)
        decisions=[]
        for now,first in ((.5,.4),(.6,.5),(.7,.6)):
            scene,_=sensor_scene(targets=((1.2,y),),now=now,first=first)
            rays=[]
            for ray in scene.rays:
                if ray.status!='return':rays.append(ray);continue
                origin=np.asarray(ray.ray_origin_xy_m);hit=np.asarray(ray.hit_xy_m)
                distance=float(np.linalg.norm(hit-origin))+float(rng.uniform(-noise,noise))
                if distance<.05:rays.append(replace(ray,status='unknown',hit_xy_m=None))
                else:rays.append(replace(ray,hit_xy_m=tuple(origin+distance*np.asarray(ray.direction_xy))))
            noisy=replace(scene,rays=tuple(rays))
            out=guide.evaluate(noisy,history,0.,0.,.02)
            decisions.append(dict(t=now,reason=out['reason'],selected=out['selected'],
                reference=out.get('reference'),boundary_confirmations=guide.start_confirmations,
                current_outer_boundary=out.get('outer_boundary_padding'),
                candidates=[{k:r.get(k) for k in ('offset_m','transition_m','reason',
                    'preview_point_gap_m','preview_return_progress_m','preview_maximum_steering_rad',
                    'outer_boundary_padding')} for r in out['candidates']]))
        rows.append(dict(target_y_m=y,uniform_range_half_width_m=noise,seed=seed,decisions=decisions))
report=dict(scope='stationary_initial_scene_three_distinct_scans_not_driving',
    noise_spec='independent uniform bounded range error; not measured C1 distribution',
    truth_given_to_controller=False,cases=rows,
    source_sha256={k:hashlib.sha256(v).hexdigest() for k,v in original.items()},
    source_unchanged=all((ROOT/k).read_bytes()==v for k,v in original.items()))
OUTPUT.write_text(json.dumps(report,indent=2,default=lambda v:v.item())+'\n',encoding='utf-8')
print(json.dumps([dict(y=r['target_y_m'],noise=r['uniform_range_half_width_m'],seed=r['seed'],
    reasons=[d['reason'] for d in r['decisions']],selected=[d['selected'] for d in r['decisions']]) for r in rows]))
