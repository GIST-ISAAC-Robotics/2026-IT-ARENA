"""저장 입력 비교용: 이산 경로점 대신 관측 경로 선분의 공통 구간을 잰다.

공간/길이 기준은 그대로다. 기본/실행 제어기에 아직 연결하지 않았다.
반대 경로는 비교에만 사용하며 조향 경로를 연장하거나 점유를 지우지 않는다.
"""
import math
import numpy as np


def merge(intervals, eps=1e-10):
    result=[]
    for lo,hi in sorted(intervals):
        if hi < lo:continue
        if result and lo <= result[-1][1]+eps:
            result[-1]=(result[-1][0],max(hi,result[-1][1]))
        else:result.append((lo,hi))
    return result


def linear_band(value, change, lower, upper):
    if abs(change)<1e-14:
        return (0.,1.) if lower<=value<=upper else None
    a,b=(lower-value)/change,(upper-value)/change
    lo,hi=max(0.,min(a,b)),min(1.,max(a,b))
    return (lo,hi) if lo<=hi else None


def circle_interval(a,step,center,radius):
    delta=a-center
    aa=float(step@step)
    if aa<=1e-20:
        return (0.,1.) if float(delta@delta)<=radius*radius else None
    bb=2*float(delta@step);cc=float(delta@delta)-radius*radius
    disc=bb*bb-4*aa*cc
    if disc<0:return None
    root=math.sqrt(disc)
    lo,hi=max(0.,(-bb-root)/(2*aa)),min(1.,(-bb+root)/(2*aa))
    return (lo,hi) if lo<=hi else None


def capsule_intervals(a,b,c,d,radius):
    """a+t(b-a), t∈[0,1]가 선분 cd의 반경 radius 안에 드는 구간."""
    step=b-a;edge=d-c;length=float(np.linalg.norm(edge))
    pieces=[]
    for center in (c,d):
        part=circle_interval(a,step,center,radius)
        if part is not None:pieces.append(part)
    if length>1e-10:
        unit=edge/length;normal=np.array([-unit[1],unit[0]])
        along=linear_band(float((a-c)@unit),float(step@unit),0.,length)
        across=linear_band(float((a-c)@normal),float(step@normal),-radius,radius)
        if along is not None and across is not None:
            lo,hi=max(along[0],across[0]),min(along[1],across[1])
            if lo<=hi:pieces.append((lo,hi))
    return merge(pieces)


def common_intervals(path,opposite,radius):
    path=np.asarray(path,dtype=float);opposite=np.asarray(opposite,dtype=float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    intervals=[]
    for i,(a,b) in enumerate(zip(path,path[1:])):
        if arc[i+1]-arc[i]<=1e-10:continue
        front=linear_band(float(a[0]),float(b[0]-a[0]),.05,float('inf'))
        if front is None:continue
        parts=[]
        for c,d in zip(opposite,opposite[1:]):
            parts.extend(capsule_intervals(a,b,c,d,radius))
        for lo,hi in merge(parts):
            lo,hi=max(lo,front[0]),min(hi,front[1])
            if hi>=lo:
                intervals.append((arc[i]+lo*(arc[i+1]-arc[i]),arc[i]+hi*(arc[i+1]-arc[i])))
    return merge(intervals),arc


def refine(discrete,config):
    """기존 성공/다른 거절은 보존하고 마지막 공통-prefix 거절만 세분한다."""
    if discrete.get('reason')!='insufficient_contiguous_common_prefix':return discrete
    own=np.asarray(discrete['preferred_info']['path_points'])
    other=np.asarray(discrete['opposite_info']['path_points'])
    nearest=int(np.argmin(np.linalg.norm(own,axis=1)))
    prefix=own[nearest:]
    from arena_autonomy.corner_corridor import projected_origin_offset
    correction=projected_origin_offset(own,nearest)
    intervals,arc=common_intervals(prefix,other,config.boundary_agreement_m)
    diagnostic=dict(reason='continuous_common_prefix_unavailable',intervals_m=intervals,
                    origin_arc_correction_m=correction)
    if not intervals:return {**discrete,'continuous_prefix':diagnostic}
    first,end=intervals[0]
    lead=first+correction
    witnesses=np.flatnonzero((arc>=first)&(arc<=end))
    # 기존 거리/길이/표현점 수는 유지하며 뒤로 접히는 대응도 거절한다.
    # 상대 경로의 연속 투영 위치가 전진하는지 비교한다.
    positions=[];other_arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(other,axis=0),axis=1))]
    for point in prefix[witnesses]:
        edge=np.diff(other,axis=0);squared=np.sum(edge*edge,axis=1)
        t=np.clip(np.sum((point-other[:-1])*edge,axis=1)/np.maximum(squared,1e-20),0.,1.)
        projections=other[:-1]+edge*t[:,None]
        idx=int(np.argmin(np.linalg.norm(projections-point,axis=1)))
        positions.append(other_arc[idx]+t[idx]*(other_arc[idx+1]-other_arc[idx]))
    supported=(0<=lead<=config.length_m and end-first>=.25 and len(witnesses)>=5
               and np.all(np.diff(positions)>=-1e-10))
    diagnostic.update(one_sided_lead_in_m=lead,shared_span_m=end-first,
                      witnesses=len(witnesses),supported=bool(supported))
    if not supported:return {**discrete,'continuous_prefix':diagnostic}
    retained=own[:nearest+1].tolist()
    for i in range(1,len(prefix)):
        if arc[i]<end-1e-10:retained.append(prefix[i].tolist())
        else:
            t=(end-arc[i-1])/(arc[i]-arc[i-1])
            retained.append((prefix[i-1]+t*(prefix[i]-prefix[i-1])).tolist());break
    return {**discrete,'supported':True,'reason':'continuous_short_observed_prefix_geometry_only',
        'short_prefix':True,'path':retained,'shared_span_m':end-first,
        'one_sided_lead_in_m':lead,'origin_arc_correction_m':correction,
        'retained_ahead_m':end,'common_distance_bound_m':config.boundary_agreement_m,
        'continuous_prefix':diagnostic,'opposite_path_used_for_steering':False,'unknown_filled':False}
