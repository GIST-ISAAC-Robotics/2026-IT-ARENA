"""진단 후보: 기본 직사각 여유 + 취득 후 이동 원판의 연속 원호 하한."""
import math
import numpy as np


def rounded_swept_limit(points, steering, wheelbase=.145, horizon=3., margin=.025,
                        length=.20, width=.15, *, fixed_padding=.055, step=.02, motion_shape='disk'):
    points=np.asarray(points,dtype=float).reshape(-1,2)
    pad=np.broadcast_to(np.asarray(margin,dtype=float),(len(points),))
    if (motion_shape not in ('disk','axis') or not np.all(np.isfinite(points)) or not np.all(np.isfinite(pad))
            or np.any(pad<fixed_padding-1e-12)
            or not all(math.isfinite(v) for v in (steering,wheelbase,horizon,length,width,fixed_padding,step))
            or not 0<step<=.04 or min(wheelbase,horizon,length,width)<=0 or fixed_padding<0):
        raise ValueError('finite geometry and fixed-plus-motion padding required')
    if not len(points):return float(horizon)
    curve=math.tan(steering)/wheelbase
    if not math.isfinite(curve):raise ValueError('finite curvature required')
    edges=np.linspace(0.,horizon,math.ceil(horizon/step)+1)
    mid=(edges[:-1]+edges[1:])/2
    yaw=curve*mid
    rx=mid-wheelbase/2 if abs(curve)<1e-12 else np.sin(yaw)/curve-wheelbase/2
    ry=np.zeros_like(mid) if abs(curve)<1e-12 else (1-np.cos(yaw))/curve
    dx=points[None,:,0]-rx[:,None];dy=points[None,:,1]-ry[:,None]
    x=np.cos(yaw[:,None])*dx+np.sin(yaw[:,None])*dy-wheelbase/2
    y=-np.sin(yaw[:,None])*dx+np.cos(yaw[:,None])*dy
    # 기존 기본25 mm + 거리 오차30 mm는 축별 직사각 여유로 유지한다.
    outside_x=np.maximum(abs(x)-length/2-fixed_padding,0.)
    outside_y=np.maximum(abs(y)-width/2-fixed_padding,0.)
    distance=(np.hypot(outside_x,outside_y) if motion_shape=='disk' else np.maximum(outside_x,outside_y))
    motion_radius=np.maximum(0.,pad-fixed_padding)
    # 후륜축을 중심으로 한 최대 점 속도로 각 구간의 표본 사이 이동을 덮는다.
    radius=math.hypot(wheelbase/2+length/2+fixed_padding,width/2+fixed_padding)
    interpolation_bound=(1.+abs(curve)*radius)*np.diff(edges)/2
    possible=np.any(distance<=motion_radius[None,:]+interpolation_bound[:,None]+1e-12,axis=1)
    hits=np.flatnonzero(possible)
    return float(edges[int(hits[0])] if len(hits) else horizon)


def sampled_rounded_limit(points,steering,wheelbase=.145,horizon=3.,margin=.025,
                           length=.20,width=.15,*,fixed_padding=.055,motion_shape='disk'):
    """기존4 cm 표본/차감은 유지하고 이동 도형만 바꾸는 분리 진단."""
    points=np.asarray(points,dtype=float).reshape(-1,2)
    padding=np.broadcast_to(np.asarray(margin,dtype=float),(len(points),))
    if motion_shape not in ('disk','axis') or np.any(padding<fixed_padding-1e-12):
        raise ValueError('declared fixed-plus-motion geometry required')
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(padding)):
        raise ValueError('finite returns required')
    curve=math.tan(steering)/wheelbase
    for distance in np.arange(0.,horizon+.001,.04):
        yaw=curve*distance
        if abs(curve)<1e-5:rx,ry=distance-wheelbase/2,0.
        else:rx,ry=math.sin(yaw)/curve-wheelbase/2,(1-math.cos(yaw))/curve
        dx,dy=points[:,0]-rx,points[:,1]-ry
        x=math.cos(yaw)*dx+math.sin(yaw)*dy-wheelbase/2
        y=-math.sin(yaw)*dx+math.cos(yaw)*dy
        ox=np.maximum(abs(x)-length/2-fixed_padding,0.)
        oy=np.maximum(abs(y)-width/2-fixed_padding,0.)
        gap=np.hypot(ox,oy) if motion_shape=='disk' else np.maximum(ox,oy)
        if np.any(gap<=np.maximum(0.,padding-fixed_padding)+1e-12):
            return max(0.,float(distance)-.04)
    return float(horizon)
