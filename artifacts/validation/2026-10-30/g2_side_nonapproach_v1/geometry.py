"""오프라인 기하 가설. 현재와 겹친 측방 불확실성에 더 접근하지 않는 표본만 비교.

현재 점 + 고정 여유는 항상 유지한다. 측방 동적 불확실성의 현재 겹침을
자유 공간이라고 부르지 않는다. 연속 sweep/물체 이동/주행 허가 증명이 아니다.
"""
import math
import numpy as np
from arena_autonomy.pass_release_trial import extrema_arc, lateral_config


def compare(points,padding,steering,distance,config):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    padding=np.asarray(padding,dtype=float)
    fixed=.025+config.measurement_padding_m
    if (padding.shape!=(len(points),) or not len(points) or not np.all(np.isfinite(points))
            or not np.all(np.isfinite(padding)) or np.any(padding<fixed-1e-12)
            or not math.isfinite(steering) or abs(steering)>config.max_steering_rad
            or not math.isfinite(distance) or not 0<=distance<=2.5):
        raise ValueError('invalid bounded geometry input')
    half=np.array([config.length_m/2,config.width_m/2])
    start_axes=np.abs(points)-half
    hard_start=np.max(start_axes-fixed,axis=1)
    start=np.max(start_axes-padding[:,None],axis=1)
    overlap=start<=0.
    # 앞면/고정 여유 안을 예외로 바꾸지 않는다. 차체 측방 이격이
    # 전방/후방 이격보다 큰 점만 측방 불확실성 가설에 속한다.
    eligible=overlap&(hard_start>0.)&(start_axes[:,1]>fixed)&(start_axes[:,1]>start_axes[:,0])
    poses=extrema_arc(steering,max(.01,distance),lateral_config(config))
    relative=points[None,:,:]-poses[:,None,:2]
    c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
    local=np.stack((c*relative[:,:,0]+s*relative[:,:,1],-s*relative[:,:,0]+c*relative[:,:,1]),axis=-1)
    axes=np.abs(local)-half
    hard=np.max(axes-fixed,axis=2)
    soft=np.max(axes-padding[None,:,None],axis=2)
    hard_hits=np.any(hard<=0.,axis=0)
    new_hits=np.any(soft<=0.,axis=0)&~overlap
    worsening=eligible&np.any(soft<start[None,:]-1e-9,axis=0)
    unsupported=overlap&~eligible
    passed=not(np.any(hard_hits)|np.any(new_hits)|np.any(worsening)|np.any(unsupported))
    return dict(sampled_geometry_candidate=bool(passed),known_base_collision_points=int(hard_hits.sum()),
        current_uncertainty_overlap_points=int(overlap.sum()),side_only_overlap_points=int(eligible.sum()),
        front_or_hard_overlap_points=int(unsupported.sum()),new_uncertainty_collision_points=int(new_hits.sum()),
        worsening_side_points=int(worsening.sum()),minimum_base_gap_m=float(hard.min()),
        worst_retained_gap_change_m=float((soft[:,eligible]-start[None,eligible]).min()) if np.any(eligible) else None,
        pose_samples=len(poses),distance_m=distance,control_authorized=False,unknown_filled=False,
        continuous_sweep_verified=False,other_object_future_motion_verified=False)
