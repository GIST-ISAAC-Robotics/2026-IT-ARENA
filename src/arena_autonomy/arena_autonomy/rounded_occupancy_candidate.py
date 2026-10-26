"""유클리드 이동 상계의 점유 후보. 기존 축별 팽창/운영 경로는 유지한다.

측정점은 거리 오차 + (상대 속력 상계 × 취득 나이) 반경의 원판으로 본다.
기본 기계 여유는 차체 좌표의 직사각형 팽창으로 그대로 남긴다. 속력은 2D
유클리드 norm의 상계이며, x/y 각각에 같은 속도 상계를 허용하는 계약과 다르다.
반환점 뒤의 물체 형상·미관측 공간·미래 진입을 인증하는 함수가 아니다.
"""
import math

import numpy as np


def point_rectangle_distance(points, steering, travel, *, wheelbase, length, width, body_padding):
    """정곡률의 축간 중심 footprint와 점의 정확한 유클리드 거리."""
    curve=math.tan(steering)/wheelbase
    yaw=curve*travel
    c,s=math.cos(yaw),math.sin(yaw)
    rear_x=travel-wheelbase/2 if abs(curve)<1e-12 else math.sin(yaw)/curve-wheelbase/2
    rear_y=0. if abs(curve)<1e-12 else 2*math.sin(yaw/2)**2/curve
    delta=np.asarray(points,dtype=float).reshape(-1,2)-[rear_x,rear_y]
    local=np.column_stack((c*delta[:,0]+s*delta[:,1]-wheelbase/2,
                           -s*delta[:,0]+c*delta[:,1]))
    outside=np.maximum(np.abs(local)-[length/2+body_padding,width/2+body_padding],0.)
    return np.linalg.norm(outside,axis=1)


def rounded_swept_limit(points, steering, wheelbase=.145, horizon=3., margin=.025,
                        length=.20, width=.15, *, body_padding=.025, step=.04):
    """원판 불확실성에 대한 연속 원호의 보수적 안전 접두 길이(평가 후보).

    호출의 margin은 기존 총 팽창량이다. body_padding은 기존 .025 m 사각형
    기계 여유로 유지하고 나머지만 원판 반경으로 쓴다. 각 구간 중앙의
    거리에서 차체 전체의 Hausdorff 이동 상계 L*반구간을 차감한다. 구간의
    빈 공간 증거가 아니며, 해당 점유 집합과의 간섭 배제만 계산한다.
    """
    p=np.asarray(points,dtype=float)
    if p.size==0:p=p.reshape(0,2)
    if p.ndim!=2 or p.shape[1]!=2 or not np.isfinite(p).all():
        raise ValueError('finite Nx2 occupied points required')
    if (not all(math.isfinite(x) for x in (steering,wheelbase,horizon,length,width,body_padding,step)) or
            not 0 < wheelbase or not 0 < length or not 0 < width or horizon < 0 or
            body_padding < 0 or not 0 < step <= .04 or abs(steering)>=math.pi/2):
        raise ValueError('invalid rounded occupancy geometry')
    padding=np.asarray(margin,dtype=float)
    if padding.ndim==0:padding=np.full(len(p),float(padding))
    if padding.shape!=(len(p),) or not np.isfinite(padding).all() or np.any(padding<body_padding):
        raise ValueError('total margin must include the preserved body padding')
    if not len(p):return float(horizon)  # 호출자는 unknown/점 개수 관문을 별도로 유지해야 한다.
    radii=padding-body_padding
    kwargs=dict(wheelbase=wheelbase,length=length,width=width,body_padding=body_padding)
    if np.any(point_rectangle_distance(p,steering,0.,**kwargs)<=radii):return 0.
    curve=math.tan(steering)/wheelbase
    # 후축 이동 길이 ds + 축간 중심/가장 먼 모서리의 회전 이동 상계.
    radius=math.hypot(length/2+body_padding,width/2+body_padding)
    lipschitz=1.+abs(curve)*(wheelbase/2+radius)
    for lo in np.arange(0.,horizon,step):
        hi=min(horizon,float(lo+step))
        lower=point_rectangle_distance(p,steering,(lo+hi)/2,**kwargs)-radii-lipschitz*(hi-lo)/2
        if np.any(lower<=0.):return float(lo)
    return float(horizon)
