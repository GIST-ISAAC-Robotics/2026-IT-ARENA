"""현재 구조 가림 양쪽 실제 벽 관측의 경로 기준 보완. 점유/빈 공간 분류가 아니다."""
import math
import numpy as np
from arena_autonomy.known_structure import ACCEPTED_HIDDEN_WIDTH_M


def rear_wall_witness(scene, config, side, fit, wall, explained_sectors):
    """기존 경로 보완 출력은 보존한다. 짧은 조각을 정적 점유로 합치지 않는다."""
    if fit is None or fit[2]<=-config.wheelbase_m/2:
        return fit,wall
    observed=observed_rear_wall_face(scene,config,side,fit,wall,explained_sectors)
    if observed is None:
        return fit,wall
    result=observed['fit']
    return result,{**wall,'observed_x_m':[result[2],result[3]],'rear_geometry_witness':observed['proof']}


def observed_rear_wall_face(scene, config, side, fit, wall, explained_sectors, *,
                            minimum_rear_span_m=.20, rear_limit_m=-.40):
    """지지대 양옆에서 실제로 보인 면이 일치할 때만 후륜축의 경로 기준을 연장한다.

    뒤쪽 짧은 면은 작은 차량 옆면일 수도 있다. 긴 벽에 붙이거나 고정 점유로
    분류하지 않는다. 원래의 고정 여유 적용 광선 목록과 가림 상태도 보존한다.
    """
    if not .20 <= minimum_rear_span_m <= 1. or not -2. <= rear_limit_m <= -.40:
        raise ValueError('bounded rear observation extent required')
    if fit is None or wall.get('reason') != 'current_connected_straight_wall_fit':
        return None
    a,b,lower,upper=fit
    if upper < .30 or upper-lower < .65:
        return None
    ids=wall['source_ray_indices']
    rays={r.source_index:r for r in scene.rays}
    if not ids or ids[0] not in rays or ids[-1] not in rays:
        return None
    # 광선 획득 순서는 좌/우에서 반대다. 차체 뒤쪽 끝에서 바깥 방향으로 검사한다.
    edge,step=min(((ids[0],-1),(ids[-1],1)), key=lambda e:rays[e[0]].hit_xy_m[0])
    sign=1 if side=='left' else -1
    norm=math.hypot(1.,a)
    actual_b=b+sign*config.wall_offset_m*norm
    shift=sign*config.wall_offset_m*a/norm
    i=edge+step
    gap=[]
    while i in rays and rays[i].status=='masked':
        gap.append(i);i+=step
    if not gap or frozenset(gap) not in explained_sectors or i not in rays or rays[i].status!='return':
        return None
    chord=math.dist(rays[edge].hit_xy_m,rays[i].hit_xy_m)
    if chord>ACCEPTED_HIDDEN_WIDTH_M:
        return None
    points=[];indices=[];last=rays[edge].hit_xy_m
    while i in rays and rays[i].status=='return':
        p=rays[i].hit_xy_m
        if (not rear_limit_m<=p[0]<last[0] or math.dist(last,p)>.05
                or abs(p[1]-a*p[0]-actual_b)/norm>.015):
            break
        points.append(p);indices.append(i);last=p;i+=step
    if len(points)<12 or points[0][0]-points[-1][0]<minimum_rear_span_m:
        return None
    x,y=np.asarray(points).T
    observed_slope=np.dot(x-x.mean(),y-y.mean())/np.dot(x-x.mean(),x-x.mean())
    new_lower=float(min(x)+shift)
    if new_lower > -config.wheelbase_m/2 or abs(math.atan(observed_slope)-math.atan(a))>.08:
        return None
    proof=dict(principal_x_m=[lower,upper],observed_rear_x_m=[float(min(x)),float(max(x))],
               rear_return_count=len(points),mask_indices=sorted(gap),shadow_chord_m=chord,
               short_piece_static=False,unknown_filled=False)
    return dict(fit=(a,b,new_lower,upper),proof=proof,source_ray_indices=indices)

