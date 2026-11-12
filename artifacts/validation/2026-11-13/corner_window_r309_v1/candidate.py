"""동일 좁은 가림 판정의 우선순위 일회 순회 후보. 경로 가설 전용."""
import importlib.util
from pathlib import Path
import numpy as np
import arena_autonomy.local_path as lp

ORIGINAL=lp.break_adjacent_occluders
path=Path(__file__).resolve().parents[1]/'straight_fragment_r306_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r306_straight_candidate',path)
straight=importlib.util.module_from_spec(spec);spec.loader.exec_module(straight)

def break_adjacent_occluders(points,broken,max_points=6,max_extent=.04,min_depth=.06,max_radial_extent=.10):
    points=np.asarray(points,dtype=float);broken=np.asarray(broken,dtype=bool)
    if points.ndim!=2 or points.shape[1]!=2 or len(broken)!=max(0,len(points)-1):
        raise ValueError('invalid wall chain shape')
    if not np.all(np.isfinite(points)):raise ValueError('nonfinite wall chain')
    alive=np.ones(len(points),dtype=bool)
    if len(points)<3:return ~alive,[]
    ranges=np.linalg.norm(points,axis=1)
    # 삭제 뒤 새로 이웃이 된 점들은 원 번호 연속 조건을 통과할 수 없다.
    # 따라서 최초 후보만 원래 edge/count/방향 우선순위로 순회할 수 있다.
    for lo,hi in lp._adjacent_cluster_candidates(np.arange(len(points)),ranges,broken,max_points,min_depth):
        if not np.all(alive[lo-1:hi+1]):continue
        before,after,cluster=points[lo-1],points[hi],points[lo:hi]
        if np.linalg.norm(after-before)>2*(.10+.035*min(ranges[lo-1],ranges[hi])):continue
        if min(ranges[lo-1],ranges[hi])-np.max(ranges[lo:hi])<=min_depth:continue
        center=cluster.mean(axis=0);direction=center/max(1e-12,np.linalg.norm(center))
        normal=np.array([-direction[1],direction[0]])
        if np.ptp(cluster@normal)>max_extent or np.ptp(cluster@direction)>max_radial_extent:continue
        alive[lo:hi]=False
    idx=np.flatnonzero(alive)
    return ~alive,[(int(a),int(b)) for a,b in zip(idx[:-1],idx[1:]) if b-a>1]

def install():
    straight.install();lp.break_adjacent_occluders=break_adjacent_occluders

def uninstall():
    straight.uninstall();lp.break_adjacent_occluders=ORIGINAL
