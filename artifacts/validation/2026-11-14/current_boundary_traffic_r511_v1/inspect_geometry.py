"""Windows 저장 센서 입력의 경계 후보만 재구성. 공통 코드/상태/장치 변경 없음."""
from pathlib import Path
import sys, json, hashlib, math
import numpy as np

# 이 PC NumPy의 2D cross 제거 호환. 옛 np.cross([x,y],[u,v])와 같은 행렬식만 사용한다.
original_cross=np.cross
def cross_compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:
        return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return original_cross(a,b,*args,**kwargs)
np.cross=cross_compat

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig, anchor_error
from arena_autonomy.local_path import local_path

sources=['moving_pass_facility_drive_r503_v1','continuous_pass_preview_drive_r513_v1']
rows=[];hashes={}
selected=set(range(90,105))|{110,130,150,170,179}
for name in sources:
    source=A.parent/name/'run1/follow_decisions.jsonl'
    hashes[name]=hashlib.sha256(source.read_bytes()).hexdigest()
    header=None
    for line in source.open(encoding='utf-8'):
        row=json.loads(line);kind,data=row['kind'],row['data']
        if kind=='runtime_evaluator_header':header=data['header']
        if kind!='runtime_plan_submitted' or data['sequence'] not in selected:continue
        cfg=FollowConfig(**header['follow_config'])
        info,scan,history=restore_call(data['input'],MotionConfig(**header['motion_config']))
        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        geometry=[]
        for side in ('left','right'):
            path,detail=local_path(points,side,cfg.wall_offset_m,horizon=1.8,
                piecewise_corner=cfg.piecewise_wall_trial,minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
                narrow_occluder_bridge=cfg.narrow_occluder_bridge)
            error=anchor_error(detail,cfg) if path is not None else None
            geometry.append(dict(side=side,**{k:v for k,v in detail.items() if k!='path_points'},
                evaluated_anchor_error_m=error,
                length_ok=path is not None and detail['boundary_observed_ahead_m']>=cfg.boundary_min_ahead_m,
                anchor_ok=error is not None and error<=cfg.boundary_anchor_error_m))
        rows.append(dict(source=name,key=[data['generation'],data['sequence']],reference_s=info['reference_s'],
            speed_mps=info['speed_mps'],occupied_count=len(points),sides=geometry,
            thresholds=dict(offset=cfg.wall_offset_m,anchor=cfg.boundary_anchor_error_m,ahead=cfg.boundary_min_ahead_m)))
(A/'geometry.json').write_text(json.dumps(freeze(dict(source_sha256=hashes,numpy_version=np.__version__,
    compatibility='2D np.cross determinant in this diagnostic process only',rows=rows)),indent=2),encoding='utf-8')
for row in rows:
    print(json.dumps(dict(source=row['source'],seq=row['key'][1],speed=row['speed_mps'],
        sides=[{k:v for k,v in s.items() if k in ('side','reason','evaluated_anchor_error_m','boundary_observed_ahead_m','length_ok','anchor_ok')} for s in row['sides']])))
