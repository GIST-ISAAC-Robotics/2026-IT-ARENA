"""진단 전용: 식별됐다고 가정한 양쪽 벽 반환에서 보수적인 도로 띠를 계산한다.

벽은 같은 관측 구간에서 y=f(x), |f'|<=slope_bound라는 조건이다. 실제 벽 분류나
기울기 상계 검증은 이 함수의 일이 아니며 운영에 연결하지 않는다. 반환 사이 물체가 없다는
뜻도 아니다. 점유/관측/독립 보호 검사는 별도로 필요하다. 빈/외삽 구간은 미확정이다.
"""
import math
import numpy as np


def side_bound(points,query_x,*,side,error_m=.03,slope_bound,maximum_gap_m=.1):
    """좌벽 하한/우벽 상한. 원판 오차의 Lipschitz 함수 지지값 e*sqrt(1+K²)를 쓴다."""
    p=np.asarray(points,dtype=float)
    q=np.asarray(query_x,dtype=float)
    if (p.ndim!=2 or p.shape[1:]!=(2,) or q.ndim!=1 or not np.isfinite(p).all() or not np.isfinite(q).all()
            or side not in ('left','right')
            or not all(math.isfinite(v) for v in (error_m,slope_bound,maximum_gap_m))
            or error_m<0 or slope_bound<0 or maximum_gap_m<=0):
        raise ValueError('finite wall points, query positions and nonnegative uncertainty/slope required')
    rows=[]
    error=error_m*math.sqrt(1+slope_bound*slope_bound)
    for x in q:
        # 오차 원판을 고려해도 query의 양쪽에 있는 반환을 요구한다. 한쪽 외삽은 하지 않는다.
        before=p[p[:,0]+error_m<=x]; after=p[p[:,0]-error_m>=x]
        if not len(before) or not len(after):
            rows.append(dict(x_m=float(x),supported=False,reason='no_robust_bracket',bound_m=None));continue
        lo=float(np.max(before[:,0]));hi=float(np.min(after[:,0]))
        # maximum_gap은 측정된 x좌표의 간격이다. 실제 가능한 간격 상한도 별도로 보고한다.
        if hi-lo>maximum_gap_m:
            rows.append(dict(x_m=float(x),supported=False,reason='observation_gap',bound_m=None,
                bracket_gap_upper_m=hi-lo+2*error_m));continue
        distance=slope_bound*np.abs(p[:,0]-x)
        values=p[:,1]-error-distance if side=='left' else p[:,1]+error+distance
        bound=float(np.max(values) if side=='left' else np.min(values))
        rows.append(dict(x_m=float(x),supported=True,reason=None,bound_m=bound,
            bracket_gap_upper_m=hi-lo+2*error_m))
    return rows


def road_strip(left,right,query_x,*,slope_bound,error_m=.03,grass_width_m=.20,
               grass_tolerance_m=.01,maximum_gap_m=.1):
    if (not math.isfinite(grass_width_m) or not math.isfinite(grass_tolerance_m)
            or grass_width_m<0 or grass_tolerance_m<0):
        raise ValueError('finite nonnegative grass width/tolerance required')
    a=side_bound(left,query_x,side='left',error_m=error_m,slope_bound=slope_bound,maximum_gap_m=maximum_gap_m)
    b=side_bound(right,query_x,side='right',error_m=error_m,slope_bound=slope_bound,maximum_gap_m=maximum_gap_m)
    # y방향 폭은 법선 방향 거리보다 크므로 기울기 상계에 맞춰 보수적으로 확보한다.
    inset=(grass_width_m+grass_tolerance_m)*math.sqrt(1+slope_bound*slope_bound)
    rows=[]
    for x,l,r in zip(query_x,a,b):
        ready=l['supported'] and r['supported']
        low=r['bound_m']+inset if ready else None
        high=l['bound_m']-inset if ready else None
        rows.append(dict(x_m=float(x),supported=bool(ready and low<=high),
            reason='missing_side_support' if not ready else 'empty_road_strip' if low>high else None,
            lower_m=low,upper_m=high,left=l,right=r))
    return dict(samples=rows,all_supported=all(r['supported'] for r in rows) and bool(rows),
        range_error_m=error_m,slope_bound=slope_bound,grass_inset_m=inset,
        conditional_on='correct same-side walls and slope bound across each observed bracket',
        driving_authorized=False)
