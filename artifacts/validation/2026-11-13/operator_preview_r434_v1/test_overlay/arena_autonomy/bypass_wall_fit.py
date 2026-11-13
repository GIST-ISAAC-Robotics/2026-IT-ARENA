"""G2 비교의 현재 직선 외벽 재관측. 과거 선은 탐색 범위이며 새 반환이 필요하다."""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid


def tracked_straight_wall(scene,config,side,remembered_centerline):
    """각 광선의 실제 연결 구간을 직선 적합한다. 점유 입력은 변경하지 않는다.

    차체가 비스듬하면 같은 외벽의 먼 부분은 전방 축의 반대편에도 보인다.
    고정 y 부호로 잘라내지 않고, 이송된 기존 벽 근처의 현재 반환으로 찾는다.
    복수 정렬 물체를 실제 벽으로 오인할 가능성은 고정 외벽 가정의 한계다.
    """
    fail=lambda reason,**extra:(None,dict(reason=reason,side=side,**extra))
    if not source_valid(scene) or side not in ('left','right'):
        return fail('tracked_wall_invalid_source')
    old=np.asarray(remembered_centerline,dtype=float)
    if old.shape!=(2,2) or not np.all(np.isfinite(old)) or abs(old[1,0]-old[0,0])<.25:
        return fail('tracked_wall_reference_unavailable')
    sign=1 if side=='left' else -1
    slope=(old[1,1]-old[0,1])/(old[1,0]-old[0,0])
    intercept=old[0,1]-slope*old[0,0]
    norm=math.sqrt(1+slope*slope)
    physical_intercept=intercept+sign*config.wall_offset_m*norm
    rays=[ray for ray in scene.rays if ray.status=='return']
    points=np.asarray([ray.hit_xy_m for ray in rays],dtype=float).reshape(-1,2)
    ids=np.array([ray.source_index for ray in rays])
    # 측정 여유 두 배 이내에서 대응 후보를 찾되, 실제 적합 잔차는 별도로 본다.
    association=2*config.measurement_padding_m
    matches=(abs(points[:,1]-slope*points[:,0]-physical_intercept)/norm<=association)&(
        points[:,0]>=-.4)&(points[:,0]<=1.8)
    joins=set()
    if config.narrow_occluder_bridge and len(points)>2:
        from arena_autonomy.local_path import narrow_occluders
        gaps=np.linalg.norm(np.diff(points,axis=0),axis=1)
        limits=.10+.035*np.minimum(np.linalg.norm(points[:-1],axis=1),np.linalg.norm(points[1:],axis=1))
        # 가까워지면 같은 작은 지주도 광선 수가 늘어난다. 고정 점 개수 대신
        # 기존 40 mm 실측 크기/깊이/그림자 길이 한도로 좁은 가림체를 구별한다.
        _,pairs=narrow_occluders(points,gaps>limits,max_points=len(points))
        # 실제 반환이 연속인 좁은 가림체만 벽 사슬에서 건너뛴다.
        # 미반환/차체 가림 광선을 메우지 않으며 해당 점의 점유는 그대로다.
        for before,after in pairs:
            if (matches[before] and matches[after]
                    and np.all(np.diff(ids[before:after+1])==1)):
                joins.add((int(ids[before]),int(ids[after])))
                matches[before+1:after]=False
    candidates=np.flatnonzero(matches)
    if len(candidates)<12:return fail('tracked_wall_current_returns_missing')
    candidates=candidates[np.argsort(ids[candidates])]
    gaps=np.linalg.norm(np.diff(points[candidates],axis=0),axis=1)
    limits=.10+.035*np.minimum(np.linalg.norm(points[candidates[:-1]],axis=1),np.linalg.norm(points[candidates[1:]],axis=1))
    breaks=(gaps>limits)|(np.diff(ids[candidates])!=1)
    for i,(before,after) in enumerate(zip(candidates,candidates[1:])):
        if (int(ids[before]),int(ids[after])) in joins:breaks[i]=False
    fits=[]
    for piece in np.split(candidates,np.flatnonzero(breaks)+1):
        part=points[piece]
        if len(part)<12 or np.ptp(part[:,0])<.30:continue
        anchors=(abs(part[:,0])<.35)&(sign*part[:,1]>.10)
        if not np.any(anchors):continue
        basis=np.c_[part[:,0],np.ones(len(part))];weights=np.ones(len(part))
        for _ in range(5):
            a,b=np.linalg.lstsq(basis*weights[:,None],part[:,1]*weights,rcond=None)[0]
            residual=abs(part[:,1]-a*part[:,0]-b)/math.sqrt(1+a*a)
            weights=np.sqrt(np.minimum(1.,.015/np.maximum(residual,1e-9)))
        if abs(math.atan(a))>math.radians(30):continue
        # 일률적인 1 cm 곡선 잔차 대신 선언된 측정 오차와 현재 점의 잔차를 대조한다.
        if np.percentile(residual,80)>config.measurement_padding_m or np.max(residual)>association:continue
        norm=math.sqrt(1+a*a)
        center_b=b-sign*config.wall_offset_m*norm
        shift_x=sign*config.wall_offset_m*a/norm
        lower=float(part[:,0].min()+shift_x);upper=float(part[:,0].max()+shift_x)
        if upper-max(lower,0.)<.25:continue
        expected=physical_intercept
        score=float(np.min(np.linalg.norm(part[anchors]-[0.,expected],axis=1)))
        fit=(float(a),float(center_b),lower,upper)
        detail=dict(slope=fit[0],intercept_m=fit[1],observed_x_m=[lower,upper],
          residual_m=float(np.max(residual)),residual_p80_m=float(np.percentile(residual,80)),
          reason='current_connected_straight_wall_fit',actual_return_count=len(piece),
          source_ray_indices=ids[piece].tolist(),association_budget_m=association,
          remembered_line_is_current_observation=False,all_returns_remain_occupied=True)
        used=[pair for pair in joins if pair[0] in ids[piece] and pair[1] in ids[piece]]
        if used:
            detail['narrow_occluders_bridged']=len(used)
            detail['wall_reference_only_not_free_space']=True
        fits.append((score,fit,detail))
    if not fits:return fail('tracked_wall_current_line_unconfirmed')
    _,fit,detail=min(fits,key=lambda item:item[0])
    return fit,detail
