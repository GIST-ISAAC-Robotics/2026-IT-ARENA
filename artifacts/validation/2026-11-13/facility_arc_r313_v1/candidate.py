"""저속 PP 안쪽 간섭의 가까운 조향 후보. 선택형 저장/제한 시험 전용."""
from contextvars import ContextVar
import importlib.util
import math
from pathlib import Path
import numpy as np
import arena_autonomy.local_follow as lf

_base=lf.LocalFollow._baseline_command
_active_base=None
_pursue=lf.pursue_path
_context=ContextVar('nearby_steering_r313',default=None)
_path=Path(__file__).resolve().parents[1]/'corner_window_r309_v1/candidate.py'
_spec=importlib.util.spec_from_file_location('r309_dependency',_path)
prior=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(prior)

def path_distance(points,path):
    a,b=path[:-1],path[1:];v=b-a;delta=points[:,None,:]-a
    t=np.clip(np.sum(delta*v,axis=2)/np.maximum(1e-12,np.sum(v*v,axis=1)),0.,1.)
    return np.min(np.linalg.norm(delta-t[:,:,None]*v,axis=2),axis=1)

def rear_arc(angle,steps,wheelbase):
    curvature=math.tan(angle)/wheelbase
    if abs(curvature)<1e-5:return np.column_stack((steps-wheelbase/2,np.zeros_like(steps)))
    yaw=curvature*steps
    return np.column_stack((np.sin(yaw)/curvature-wheelbase/2,(1-np.cos(yaw))/curvature))

def choose(path,points,padding,cfg,speed,desired):
    detail=dict(applied=False,reason='outside_low_speed_scope',unknown_filled=False,occupancy_preserved=True)
    if not (math.isfinite(speed) and 0<=speed<=.25 and math.isfinite(desired)):
        return desired,detail
    path=np.asarray(path,dtype=float);points=np.asarray(points,dtype=float)
    if (path.ndim!=2 or path.shape[1]!=2 or len(path)<3 or not np.all(np.isfinite(path))
            or points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.all(np.isfinite(points))):
        return desired,{**detail,'reason':'invalid_geometry'}
    clearance=lambda angle:lf.swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=padding,
                                          length=cfg.length_m,width=cfg.width_m)
    original=clearance(desired)
    detail.update(original_angle_rad=desired,original_clearance_m=original,reason='original_clear')
    if original>cfg.hold_clearance_m or abs(desired)<1e-9:return desired,detail
    steps=np.linspace(0.,cfg.resume_clearance_m+.04,33)
    # 기존 경로에 가까운 작은 조향 완화부터 확인한다. 반대 방향/더 안쪽 꺾기는 만들지 않는다.
    options=[]
    for reduction in (.015,.03,.045,.06,.075,.09,.105,.12):
        angle=math.copysign(max(0.,abs(desired)-reduction),desired)
        error=float(np.max(path_distance(rear_arc(angle,steps,cfg.wheelbase_m),path)))
        if error>cfg.boundary_agreement_m:
            options.append(dict(angle_rad=angle,path_error_m=error,reason='route_diverges'))
            continue
        gap=clearance(angle)
        options.append(dict(angle_rad=angle,path_error_m=error,clearance_m=gap))
        if gap>=cfg.resume_clearance_m:
            return angle,{**detail,'applied':True,'reason':'nearby_clear_arc','angle_rad':angle,
                          'clearance_m':gap,'path_error_m':error,'attempts':options}
        if angle==0.:break
    return desired,{**detail,'reason':'no_nearby_clear_arc','attempts':options}

def pursue(path,speed,cfg,age,**kwargs):
    result=_pursue(path,speed,cfg,age,**kwargs)
    context=_context.get()
    if context is None:return result
    owner,scene,detail=context
    if not (math.isfinite(speed) and 0<=speed<=.25):return result
    if owner.path_source!=scene.source_first_s:
        detail.update(applied=False,reason='not_current_observed_path')
        return result
    if cfg.occupancy_timing_mode!='per_ray_candidate':
        detail.update(applied=False,reason='unsupported_timing_policy')
        return result
    padding=lf.per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
    angle,evidence=choose(path,scene.occupied_endpoints_xy_m,padding,cfg,speed,result[1])
    detail.update(evidence)
    # 원 속도 상한/남은 길이는 유지한다. 발행각3rad/s 제한·현재/직전각 점유 검사는 원 처리다.
    return result[0],angle,result[2]

def baseline(self,scene,history,speed,side,previous_steering=0.,control_dt=.02,**kwargs):
    detail={};token=_context.set((self,scene,detail))
    try:
        result=_active_base(self,scene,history,speed,side,previous_steering,control_dt,**kwargs)
    finally:_context.reset(token)
    if detail:result=result[0],result[1],{**result[2],'nearby_steering_candidate':detail}
    return result

def install():
    global _active_base
    prior.install()
    _active_base=lf.LocalFollow._baseline_command
    # r306은 전용 globals로 함수를 만든다. 그 구현을 보존한 채 PP 호출만 감싼다.
    _active_base.__globals__['pursue_path']=pursue
    lf.LocalFollow._baseline_command=baseline

def uninstall():
    lf.LocalFollow._baseline_command=_base;lf.pursue_path=_pursue;prior.uninstall()
