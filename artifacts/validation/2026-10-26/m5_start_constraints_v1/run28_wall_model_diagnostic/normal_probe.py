"""진단 전용: 공통 벽 법선에서 거리 오차를 설명하는 중심 구간을 찾는다.

양쪽 반환이 알려진 폭의 평행 벽이라는 조건부 모형이다. 물체 분류/도로 허가가 아니며
시간·장착·운동 오차를 대신하지 않는다. 유한 각도 탐색의 실패는 전역 불가능 증명이 아니다.
"""
import math
import numpy as np


def _inputs(points, directions, sides, wall_half_m, error_m):
    p, d, s = (np.asarray(x, dtype=float) for x in (points,directions,sides))
    if (p.ndim != 2 or p.shape[1:] != (2,) or d.shape != p.shape or s.shape != (len(p),)
            or not all(np.isfinite(x).all() for x in (p,d,s))
            or not np.isin(s,[-1.,1.]).all()
            or not np.allclose(np.linalg.norm(d,axis=1),1.,atol=1e-10,rtol=0.)):
        raise ValueError('finite points, unit ray directions and signed sides required')
    if (not math.isfinite(wall_half_m) or wall_half_m <= 0
            or not math.isfinite(error_m) or error_m < 0):
        raise ValueError('finite positive half width and nonnegative range error required')
    return p,d,s


def _bounds(p,d,s,theta,half,error,radial):
    normal = np.array([-math.sin(theta),math.cos(theta)])
    centers = p@normal-s*half
    radius = error*np.abs(d@normal) if radial else np.full(len(p),error)
    lower,upper = centers-radius,centers+radius
    il,iu = int(np.argmax(lower)),int(np.argmin(upper))
    return float(lower[il]),float(upper[iu]),il,iu


def normal_interval(points,directions,sides,theta,*,wall_half_m=.425,error_m=.03,radial=True):
    p,d,s = _inputs(points,directions,sides,wall_half_m,error_m)
    if not math.isfinite(theta) or type(radial) is not bool:
        raise ValueError('finite normal angle and explicit radial bool required')
    if not np.any(s==1) or not np.any(s==-1):
        return dict(status='missing_side',center_interval_m=None,gap_m=None)
    lo,hi,il,iu = _bounds(p,d,s,theta,wall_half_m,error_m,radial)
    return dict(status='compatible' if lo<=hi else 'inconsistent',
        center_interval_m=[lo,hi] if lo<=hi else None,lower_m=lo,upper_m=hi,gap_m=lo-hi,
        limiting_lower_index=il,limiting_upper_index=iu)


def search_normal(points,directions,sides,*,theta_lo=-.08,theta_hi=.08,wall_half_m=.425,
                  error_m=.03,radial=True):
    p,d,s = _inputs(points,directions,sides,wall_half_m,error_m)
    if (not math.isfinite(theta_lo) or not math.isfinite(theta_hi) or theta_hi<=theta_lo
            or theta_hi-theta_lo>math.pi/2 or type(radial) is not bool):
        raise ValueError('ordered bounded local angle search required')
    common=dict(driving_authorized=False,scope='finite angle search plus local refinement; no global infeasibility proof',
                left_points=int(sum(s==1)),right_points=int(sum(s==-1)),radial=radial,
                theta_range_rad=[theta_lo,theta_hi],error_m=error_m,wall_half_m=wall_half_m)
    if not np.any(s==1) or not np.any(s==-1):
        return dict(common,status='missing_side',center_interval_m=None)
    def objective(theta):
        lo,hi,_,_ = _bounds(p,d,s,theta,wall_half_m,error_m,radial)
        return lo-hi
    grid=np.linspace(theta_lo,theta_hi,401)
    scores=np.array([objective(t) for t in grid])
    candidates=[(float(scores[i]),float(grid[i])) for i in np.argsort(scores)[:3]]
    # 최소 부근의 작은 각도 구간만 정밀화한다. 미탐색 모형을 가능/불가능으로 확정하지 않는다.
    ratio=(math.sqrt(5)-1)/2
    for index in np.argsort(scores)[:3]:
        a,b=grid[max(0,index-1)],grid[min(len(grid)-1,index+1)]
        c,e=b-ratio*(b-a),a+ratio*(b-a)
        fc,fe=objective(c),objective(e)
        for _ in range(64):
            if fc<fe:
                b,e,fe=e,c,fc; c=b-ratio*(b-a); fc=objective(c)
            else:
                a,c,fc=c,e,fe; e=a+ratio*(b-a); fe=objective(e)
        theta=(a+b)/2
        candidates.append((objective(theta),float(theta)))
    _,theta=min(candidates)
    answer=normal_interval(p,d,s,theta,wall_half_m=wall_half_m,error_m=error_m,radial=radial)
    return dict(common,**answer,theta_rad=theta,sampled_compatible_count=int(sum(scores<=0)))
