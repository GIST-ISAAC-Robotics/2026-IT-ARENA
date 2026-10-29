"""호출 내 캐시의 동일 출력 비교와 정지 자세의 추가 합성 관측. 원래 실패 재생은 아니다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import types
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy import bypass_entry_trial as current
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry import case_specs, sensor_scene
from probe_bypass_entry_sequence import render_scan

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
OUTPUT=BASE/'cache_terminal_v2'
OUTPUT.mkdir(exist_ok=False)
name='arena_autonomy._entry_sequence5_comparison'
old=types.ModuleType(name)
sys.modules[name]=old
relative='src/arena_autonomy/arena_autonomy/bypass_entry_trial.py'
with zipfile.ZipFile(BASE/'sequence5/sources.zip') as archive:
    before=archive.read(relative)
exec(compile(before,'sequence5/'+relative,'exec'),old.__dict__)
after=(ROOT/relative).read_bytes()
equivalence=[]
for label,kwargs in case_specs():
    scene,_=sensor_scene(**kwargs)
    for speed,steer in ((0.,0.),(.2,.04),(.35,-.07)):
        began=time.perf_counter();a=old.compare_entries(scene,scene.t_ref_s,speed,steer,.02)
        old_ms=1000*(time.perf_counter()-began)
        began=time.perf_counter();b=current.compare_entries(scene,scene.t_ref_s,speed,steer,.02)
        new_ms=1000*(time.perf_counter()-began)
        equal=a==b
        equivalence.append(dict(case=label,speed=speed,steer=steer,equal=equal,old_ms=old_ms,new_ms=new_ms))
        if not equal:raise AssertionError(f'cache output changed: {label} {speed} {steer}')

terminal=[]
for file in sorted((BASE/'sequence5').glob('y*_restart*.json')):
    saved=json.loads(file.read_text(encoding='utf-8'));last=saved['records'][-1]
    x,y,yaw=last['evaluation']['center_pose'];now=last['t']
    rear=(x-.0675*math.cos(yaw),y-.0675*math.sin(yaw),yaw)
    physical=[dict(t=0.,rear_pose=rear,speed=0.,steering=0.),dict(t=now,rear_pose=rear,speed=0.,steering=0.)]
    raw=render_scan(physical,now,saved['target_y'],saved['target_restarts'])
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(now-.3,now+.0001,.01):
        history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    scene=motion_scene(raw,history,now,timing_verified=True,rear_half_angle_deg=30.)
    cfg=FollowConfig();reference,evidence=current.straight_reference(scene,cfg)
    decision=current.compare_entries(scene,now,0.,last['safe'][1],.02)
    points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
    padding=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
    angles=[dict(angle=float(angle),clearance=swept_limit(points,float(angle),cfg.wheelbase_m,
        horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m)) for angle in np.linspace(-.3,.3,13)]
    fig,ax=plt.subplots(figsize=(9,5))
    ax.scatter(points[:,0],points[:,1],s=4,c='black',label='current return')
    walls={}
    for side,sign,color in (('left',1,'tab:blue'),('right',-1,'tab:orange')):
        path,detail=local_path(points[sign*points[:,1]>.10],side,offset=cfg.wall_offset_m,horizon=1.8)
        walls[side]=detail
        if path is not None:ax.plot(path[:,0],path[:,1],color=color,label=side+' shifted fit')
    for row in decision['candidates']:
        if 'path_points' in row and row['offset_m']*saved['lateral_displacement_m']>0:
            path=np.asarray(row['path_points']);ax.plot(path[:,0],path[:,1],alpha=.15,color='green')
    ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,fill=False,color='red'))
    ax.set(xlim=(-.4,1.4),ylim=(-.65,.65),xlabel='ego x (m)',ylabel='ego y (m)',title=file.stem+' stationary rescan')
    ax.axis('equal');ax.set_xlim(-.4,1.4);ax.set_ylim(-.65,.65);ax.grid();ax.legend()
    fig.tight_layout();fig.savefig(OUTPUT/(file.stem+'.png'),dpi=140);plt.close(fig)
    terminal.append(dict(case=file.stem,scope='stationary_terminal_rescan_not_original_replay',
        reference=evidence,walls=walls,selected=decision['selected'],previous=last['safe'][1],
        angles=angles,candidates=[{k:r.get(k) for k in ('offset_m','transition_m','reason','steering_rad',
        'command_clearance_m','preview_return_progress_m')} for r in decision['candidates']]))
report=dict(equivalence=equivalence,all_equal=all(r['equal'] for r in equivalence),terminal=terminal,
    before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=hashlib.sha256(after).hexdigest(),
    source_unchanged=(ROOT/relative).read_bytes()==after,
    timing_scope='single alternating function samples on PC; not realtime or Jetson')
def numpy_scalar(value):
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)
(OUTPUT/'report.json').write_text(json.dumps(report,indent=2,default=numpy_scalar)+'\n',encoding='utf-8')
print(json.dumps(dict(equal=len(equivalence),old_median_ms=float(np.median([r['old_ms'] for r in equivalence])),
    new_median_ms=float(np.median([r['new_ms'] for r in equivalence])),source_unchanged=report['source_unchanged'])))
