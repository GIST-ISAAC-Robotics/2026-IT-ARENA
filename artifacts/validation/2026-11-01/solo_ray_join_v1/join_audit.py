"""연결 후보를 저장 입력에 메모리로만 적용하고, 연결 구간이 실제 벽인지 평가 정답으로 대조한다.

정답 벽/자세는 이 스크립트의 판정에만 쓴다. 제어 입력에는 들어가지 않는다. 런타임 소스 변경 없음.
호출: join_audit.py <variant> <run-folder> <출력.json>
"""
import ast
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from shapely.geometry import Point

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(HERE.parent/'solo_horizon_probe_v1'))
from arena_autonomy import local_follow as module
from arena_autonomy.follow_trace import replay_call,restore_call,freeze,equal_output,comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from sampling import supported_gaps
from surface_contact_eval import SurfaceMap


def chain_gaps(points,gaps,limits,scene,horizon,padding=.03,base=6,min_span=.30,heading_deg=None):
    """진행 방향 근거리 연속 광선의 직선에 다음 두 실제 점이 놓이는지 본다."""
    accepted=np.zeros(len(gaps),dtype=bool)
    source={tuple(r.hit_xy_m):r.source_index for r in scene.rays if r.status=='return'}
    ids=np.array([source.get(tuple(p),-10000) for p in points])
    ranges=np.linalg.norm(points,axis=1)
    for i in np.flatnonzero(gaps>limits):
        if i+1<base or i+2>=len(points) or ranges[i+1]>horizon:continue
        steps=np.diff(ids[i+1-base:i+3])
        if not (np.all(steps==1) or np.all(steps==-1)):continue
        part=points[i+1-base:i+1]
        if np.linalg.norm(part[-1]-part[0])<min_span:continue
        center=part.mean(axis=0);_,_,vh=np.linalg.svd(part-center,full_matrices=False)
        tangent=vh[0] if vh[0]@(part[-1]-part[0])>0 else -vh[0]
        if heading_deg is not None and (tangent[0]<=0 or abs(tangent[1])>math.sin(math.radians(heading_deg))):continue
        window=points[i+1-base:i+3]-center
        if np.max(np.abs(window@[-tangent[1],tangent[0]]))>padding:continue
        if np.any(np.diff(window@tangent)<=0):continue
        accepted[i]=True
    return accepted


VARIANTS=dict(gpt_v1=lambda *a:supported_gaps(*a),
              chain_3cm=lambda *a:chain_gaps(*a),
              chain_2cm=lambda *a:chain_gaps(*a,padding=.02),
              chain_h15=lambda *a:chain_gaps(*a,heading_deg=15.),
              chain_h20=lambda *a:chain_gaps(*a,heading_deg=20.),
              chain_h30=lambda *a:chain_gaps(*a,heading_deg=30.),
              off=lambda p,g,l,s,h:np.zeros(len(g),dtype=bool))
name,folder,output=sys.argv[1:4]
folder=Path(folder);chosen=VARIANTS[name]
surface=SurfaceMap.from_world(folder/'runtime_world.sdf')
poses=np.array([json.loads(line) for line in (folder/'ego_poses.jsonl').read_text(encoding='utf-8').splitlines()])
poses=poses[np.argsort(poses[:,0],kind='stable')]
walls=surface.walls
scene=None;now=None;activity=Counter();mistakes=[];seen=set()


def pose_at(t):
    i=min(int(np.searchsorted(poses[:,0],t)),len(poses)-1)
    return poses[i][1:4]


def world(xy,pose):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return x+c*xy[...,0]-s*xy[...,1],y+s*xy[...,0]+c*xy[...,1]


def along_wall(a,b,pose):
    """연결한 두 점 사이가 모두 실제 벽 표면 가까이에 있는가. 평가 전용."""
    worst=0.
    for f in np.linspace(0.,1.,9):
        wx,wy=world(np.asarray(a)*(1-f)+np.asarray(b)*f,pose)
        worst=max(worst,walls.distance(Point(wx,wy)))
    return worst


def break_indices(points,gaps,limits,horizon):
    base=gaps>limits
    if scene is None or horizon<=1.8:return base.nonzero()[0]
    joins=chosen(points,gaps,limits,scene,horizon)
    key=(scene.source_first_s,tuple(np.flatnonzero(base)[:4]))
    if key not in seen:
        seen.add(key);pose=pose_at(now)
        ranges=np.minimum(np.linalg.norm(points[:-1],axis=1),np.linalg.norm(points[1:],axis=1))
        for i in np.flatnonzero(base&(ranges<=horizon)&(points[:-1,0]>.3)):
            distance=along_wall(points[i],points[i+1],pose)
            real=distance<=.05
            activity[('accepted' if joins[i] else 'rejected')+('_on_wall' if real else '_off_wall')]+=1
            if joins[i] and not real:
                mistakes.append(dict(t=now,before=points[i].tolist(),after=points[i+1].tolist(),wall_distance_m=float(distance)))
    return ((gaps>limits)&~joins).nonzero()[0]


original=module.local_path
tree=ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_path.py').read_text(encoding='utf-8'))
fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='local_path')
modified=0
for statement in fn.body:
    if isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='breaks' for t in statement.targets):
        statement.value=ast.parse('break_indices(points,gaps,limits,horizon)',mode='eval').body;modified+=1
assert modified==1
namespace={**original.__globals__,'break_indices':break_indices}
exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'diagnostic_join_path','exec'),namespace)
candidate=namespace['local_path']

source=folder/'follow_decisions.jsonl'
metrics_path=folder.parent/'run1_metrics.json'
metrics=json.loads(metrics_path.read_bytes()) if metrics_path.exists() else {}
begin=metrics.get('first_green_sample_s') or 0.
lap=metrics.get('lap_s_from_first_green_sample')
end=begin+lap if lap else float('inf')
counts=Counter();changes=[];last=-1;footer=None;started=time.time()
hit_check=[]
try:
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e=json.loads(line);d=e['data'];kind=e['kind']
            if e['seq']!=last+1 or footer is not None:raise ValueError('trace order')
            last=e['seq']
            if kind=='header':
                cfg=module.FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                old=module.LocalFollow(cfg);new=module.LocalFollow(cfg)
            elif kind=='reset':old.reset();new.reset()
            elif kind=='footer':footer=d
            elif kind=='call':
                module.local_path=original;a=replay_call(old,d['input'],motion)
                if not equal_output(comparable_decision(freeze(a)),comparable_decision(d['output'])):raise ValueError('baseline mismatch '+str(last))
                counts['baseline_matched']+=1
                inp,scan,history=restore_call(d['input'],motion)
                now=inp['reference_s']
                scene=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
                if len(hit_check)<40 and counts['baseline_matched']%50==0:
                    pose=pose_at(now);points=np.asarray(scene.occupied_endpoints_xy_m);points=points[::20]
                    hit_check.append(float(np.median([walls.distance(Point(*world(p,pose))) for p in points])))
                module.local_path=candidate;b=replay_call(new,d['input'],motion)
                t=inp['reference_s'];dt=inp['dt_s']
                if not begin<=t<=end:continue
                counts['lap_calls']+=1
                for tag,value in [('baseline',a[0]),('candidate',b[0])]:
                    counts[tag+'_target_integral_m']+=value*dt
                    counts[tag+'_target_ge_2p6_s']+=dt*(value>=2.6)
                    counts[tag+'_peak_mps']=max(counts[tag+'_peak_mps'],value)
                    detail=(a if tag=='baseline' else b)[2]
                    ahead=detail.get('observed_path_ahead_m')
                    if ahead is not None:
                        counts[tag+'_path_ge_2p6_s']+=dt*(ahead>=2.6)
                if abs(a[0]-b[0])>1e-8 or abs(a[1]-b[1])>1e-8:
                    counts['commands_changed']+=1;counts['new_zero']+=int(a[0]>.001>=b[0]);counts['zero_released']+=int(b[0]>.001>=a[0])
                    counts['speed_up']+=int(b[0]>a[0]+1e-8);counts['speed_down']+=int(b[0]<a[0]-1e-8)
                    changes.append(dict(t=t,old_speed=a[0],new_speed=b[0],old_steering=a[1],new_steering=b[1],old_reason=a[2].get('reason'),new_reason=b[2].get('reason')))
            else:raise ValueError('unknown event')
finally:module.local_path=original
output_data=dict(scope='연결 후보 메모리 진단. 평가 정답은 판정에만 사용. 새 주행 예측 아님.',variant=name,run=folder.as_posix(),counts=dict(counts),activity=dict(activity),
    mistakes=mistakes[:40],mistake_count=len(mistakes),changes=changes,median_hit_to_wall_m=float(np.median(hit_check)) if hit_check else None,
    max_steering_change=max((abs(c['new_steering']-c['old_steering']) for c in changes),default=0.),
    input_sha256=sha256(source.read_bytes()).hexdigest(),elapsed_s=time.time()-started)
Path(output).write_text(json.dumps(freeze(output_data),ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps({k:output_data[k] for k in ('variant','counts','activity','mistake_count','median_hit_to_wall_m','max_steering_change','elapsed_s')},ensure_ascii=False))
