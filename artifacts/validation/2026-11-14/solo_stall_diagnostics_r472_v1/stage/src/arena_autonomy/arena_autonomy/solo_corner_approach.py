"""현재 전방 횡단면까지 남은 거리로 직각 코너의 감속을 앞당기는 제한 비교."""
import math,statistics

def crossing_face(rays,width):
    # 실제 연속 반환만 사용한다. 앞쪽 ±(차체 반폭+3cm) 띠를 가로지르는 횡단면.
    bands=[];group=[];last=None
    def end():
        if len(group)<5:return
        xs=[x for x,y in group];ys=[y for x,y in group]
        if min(ys)>-width/2 or max(ys)<width/2:return
        if max(xs)-min(xs)>.06+.15*(max(ys)-min(ys)):return
        if max(abs(ys[i]-ys[i-1]) for i in range(1,len(ys)))>.055:return
        bands.append(statistics.median(xs))
    for r in rays:
        hit=r.hit_xy_m if r.status=='return' else None
        if hit is None or not (.30<=hit[0]<=4.5 and abs(hit[1])<=width/2+.03):
            if group:end();group=[]
            last=None;continue
        if last is not None and (r.source_index!=last+1 or math.dist(hit,group[-1])>.065):end();group=[]
        group.append(hit);last=r.source_index
    if group:end()
    return min(bands) if bands else None

def approach_speed(original,scene,cfg):
    v,steer,detail=original
    if not (cfg.solo_route_only and cfg.solo_straight_budget_trial and v>1.2 and detail.get('reason')=='local_follow'):
        return v,None
    age=scene.t_ref_s-scene.source_first_s
    if not 0<=age<.30:return v,None
    distance=crossing_face(scene.rays,cfg.width_m)
    if distance is None:return v,None
    # 외벽까지 거리에서 중심선 벽 오프셋과 같은 반경의 전환 시작 위치를 차감한다.
    # 평가 지도 없이 현재 횡단면으로만 작동. 이 횡단면이 차량이면 기존 낮은 추종/정지보다 올리지 않는다.
    a=1.5;turn=1.2;remain=max(0.,distance-2*cfg.wall_offset_m-.06);delay=age+.10
    cap=max(turn,math.sqrt((a*delay)**2+turn**2+2*a*remain)-a*delay)
    return min(v,cap),dict(distance_m=distance,remaining_m=remain,cap_mps=cap)
