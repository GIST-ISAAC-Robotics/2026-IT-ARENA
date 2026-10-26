"""원문 각도 metadata를 보존하는 순환 인접성 시제품. ROS/운영 제어 미연결.

LocalScene의 투영된 방향만 보고 한 바퀴 입력이었다고 추정하지 않는다. 원문을
scene_from_scan과 함께 해석하고, 끝점을 중복하지 않는 정확한 한 바퀴만 증명 객체로 묶는다.
이 증명은 배열 구조에 관한 것이며 실물 C1의 취득 시각 정확성/감지 능력 증명이 아니다.
"""
from dataclasses import dataclass
import math
import numpy as np
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.observation_support import depth_support
from arena_autonomy.m3_execution_trial import _both_sides_returned as legacy_both_returned


@dataclass(frozen=True)
class CycleScene:
    scene: object
    source_angle_min: float
    source_angle_increment: float
    ray_count: int
    full_turn: bool
    rejection: str | None


def scene_with_cycle(ranges, **kwargs):
    values=tuple(ranges)
    scene=scene_from_scan(values,**kwargs)
    minimum=float(kwargs['angle_min'])
    increment=float(kwargs['angle_increment'])
    count=len(values)
    # 수치 반올림 여유만 허용한다. 누락된 한 광선의 각도를 메우는 허용치가 아니다.
    # ROS LaserScan angle_increment는 float32다. 한 바퀴의 누적 반올림만
    # 허용한다. 최대 8192광선에서도 한 광선 간격보다 1000배 이상 작다.
    tol=max(1e-9,math.tau*np.finfo(np.float32).eps)
    rejection=None
    if count<3 or abs(count*increment-math.tau)>tol:
        rejection='not_endpoint_excluded_full_turn'
    elif any(b.acquired_at_s<=a.acquired_at_s for a,b in zip(scene.rays,scene.rays[1:])):
        rejection='acquisition_not_strictly_ordered'
    return CycleScene(scene,minimum,increment,count,rejection is None,rejection)


def adjacent_sources(scene,lo,hi,cycle):
    """평범한 이웃 규칙 + 동일 장면의 검증된 첫/끝 한 쌍. 상태/여유는 여기서 판단하지 않는다."""
    indices=np.asarray([r.source_index for r in scene.rays])
    delta=np.abs(indices[lo]-indices[hi])
    adjacent=delta<=1
    if (cycle is not None and cycle.scene is scene and cycle.full_turn
        and cycle.ray_count==len(scene.rays)
        and np.array_equal(indices,np.arange(cycle.ray_count))):
        adjacent |= delta==cycle.ray_count-1
    return adjacent


def cycle_depth_support(scene,queries,padding,*,cycle=None):
    """순수 비교용 전 광선 기준식. 기본 cycle=None은 기존 depth_support 그대로 호출.

공간적 이웃으로 볼 수 있어도 양쪽 반환·각도 간격·원점·거리·각 광선의 기존 여유를
모두 만족해야 한다. 취득시각을 새로 만들거나 가장 최근 값으로 바꾸지 않는다.
"""
    if cycle is None or cycle.scene is not scene or not cycle.full_turn:
        return depth_support(scene,queries,padding)
    q=np.asarray(queries,dtype=float).reshape(-1,2)
    result=np.zeros(len(q),dtype=bool)
    if not len(q):return result
    rays=scene.rays
    origins=np.array([r.ray_origin_xy_m for r in rays])
    directions=np.array([r.direction_xy for r in rays])
    valid=np.array([r.status=='return' for r in rays])
    depths=np.array([math.dist(r.ray_origin_xy_m,r.hit_xy_m) if r.status=='return' else -1. for r in rays])
    pad=np.asarray(padding,dtype=float)
    if pad.shape!=(len(rays),) or not np.all(np.isfinite(pad)) or np.any(pad<0):
        raise ValueError('finite nonnegative per-ray padding required')
    for k in range(0,len(q),128):
        delta=q[k:k+128,None,:]-origins[None,:,:]
        norm=np.linalg.norm(delta,axis=2)
        angles=np.arctan2(directions[None,:,0]*delta[:,:,1]-directions[None,:,1]*delta[:,:,0],
                          np.sum(directions[None,:,:]*delta,axis=2))
        low=np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf)
        high=np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf)
        lo,hi=np.argmin(low,axis=1),np.argmin(high,axis=1)
        row=np.arange(len(delta))
        result[k:k+len(delta)]=(np.all(norm>1e-9,axis=1) & np.isfinite(low[row,lo])
            & np.isfinite(high[row,hi]) & (low[row,lo]+high[row,hi]<=.025)
            & adjacent_sources(scene,lo,hi,cycle) & valid[lo] & valid[hi]
            & (depths[lo]-norm[row,lo]-pad[lo]>0) & (depths[hi]-norm[row,hi]-pad[hi]>0))
    return result


def cycle_both_returned(scene,points,*,cycle=None):
    """M3 반환 종류 검사의 같은 순환 계약. 거리/여유 검사는 호출자의 별도 관문이다."""
    if cycle is None or cycle.scene is not scene or not cycle.full_turn:
        return legacy_both_returned(scene,points)
    q=np.asarray(points,dtype=float).reshape(-1,2)
    result=np.zeros(len(q),dtype=bool)
    if not len(q):return result
    origins=np.array([r.ray_origin_xy_m for r in scene.rays])
    directions=np.array([r.direction_xy for r in scene.rays])
    returned=np.array([r.status=='return' for r in scene.rays])
    for k in range(0,len(q),128):
        delta=q[k:k+128,None,:]-origins[None,:,:]
        angles=np.arctan2(directions[None,:,0]*delta[:,:,1]-directions[None,:,1]*delta[:,:,0],
                          np.sum(directions[None,:,:]*delta,axis=2))
        low=np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf)
        high=np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf)
        lo,hi=np.argmin(low,axis=1),np.argmin(high,axis=1);row=np.arange(len(delta))
        result[k:k+len(delta)]=(np.isfinite(low[row,lo])&np.isfinite(high[row,hi])
            &(low[row,lo]+high[row,hi]<=.025)&adjacent_sources(scene,lo,hi,cycle)&returned[lo]&returned[hi])
    return result
