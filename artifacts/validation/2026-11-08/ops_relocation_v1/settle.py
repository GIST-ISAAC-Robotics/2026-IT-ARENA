"""평가 전용. STOP 뒤의 제동 과정과 0.5초 정지 유지를 분리한다."""
import math

def hold_window(rows, duration=.5, peak_limit=.05, max_gap=.12, drift_limit=.02):
    """최근 duration 이상 연속 창. 유효성/최대속도/변위 모두 확인한다."""
    if len(rows)<2:return None
    b=rows[-1][0]
    i=len(rows)-1
    while i>0 and b-rows[i][0]<duration:i-=1
    w=rows[i:]
    if b-w[0][0]<duration-1e-9:return None
    if not all(all(math.isfinite(v) for v in r[:3]) for r in w):return None
    ds=[y[0]-x[0] for x,y in zip(w,w[1:])]
    if not ds or min(ds)<=0 or max(ds)>max_gap:return None
    vs=[math.dist(x[1:3],y[1:3])/dt for x,y,dt in zip(w,w[1:],ds)]
    drift=max(math.dist(w[0][1:3],r[1:3]) for r in w)
    return dict(from_sim_s=w[0][0],to_sim_s=b,duration_s=b-w[0][0],samples=len(w),
                peak_speed_mps=max(vs),max_pose_gap_s=max(ds),drift_m=drift,
                passed=max(vs)<=peak_limit and drift<=drift_limit)
