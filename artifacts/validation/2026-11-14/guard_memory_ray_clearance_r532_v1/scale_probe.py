"""20초/200회 반복 합성의 활성집합/평가비용. Jetson·ROS 처리율 검증은 아니다."""
from pathlib import Path
import json,sys,time
import numpy as np
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface'),str(A/'stage/tests'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
from test_guard_memory_command import seeded,PARTIAL
from test_independent_guard_route import scan
from arena_autonomy.guard_memory_command import limited_memory_command
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scan_input import project_scan
from arena_autonomy.follow_trace import freeze
h=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=.5))
p,_=seeded(h=h);original=p.guard_memory.original.copy();last=-1;rows=[];previous=None
for tick in range(5,206):
    now=tick/10
    for sample in range(last+1,round(now*100)+1):
        h.add_mean_wheel(sample/100,0.);h.add_gyro(sample/100,0.)
    last=round(now*100)
    raw=scan(now-.1,[2.]*500)
    missing=np.arange(300 if tick%2==0 else 310,305 if tick%2==0 else 315)
    for index in missing:raw.ranges[int(index)]=float('nan')
    points,ids,_=project_scan(raw,h,now,timing_verified=True)
    if previous is None:
        previous=(points,ids,now,now-.1);continue
    old,old_ids,reference,source=previous
    selected=np.flatnonzero(np.isin(old_ids,missing))
    assert len(selected)==5
    started=time.perf_counter()
    p.guard_memory.retain(old[selected],reference,source,('laser_frame',0),old_ids[selected])
    result=limited_memory_command(p,PARTIAL,raw,points,h,now,.8,0.,0.,0.,.135,.20,.15)
    elapsed=(time.perf_counter()-started)*1000
    assert result['accepted'] and result['speed_cap_mps']==.1 and p.latched
    assert np.array_equal(p.guard_memory.original,original)
    assert sum(not np.isfinite(v) for v in raw.ranges)==5
    rows.append(dict(now_s=now,active=len(p.guard_memory.points),original_points=len(p.guard_memory.seen),
        handed_off=result['active_points_reobserved'],evaluation_ms=elapsed))
    previous=(points,ids,now,now-.1)
report=dict(scope='200 sequential synthetic memory-candidate calls over20s; alternating5 actual rays disappear/reappear; acquisition and raw projection outside measured interval; no ROS/Jetson/full guard timing',
    calls=len(rows),duration_s=20.,max_active_points=max(r['active'] for r in rows),
    original_observation_points=len(p.guard_memory.seen),active_final=len(p.guard_memory.points),
    max_evaluation_ms=max(r['evaluation_ms'] for r in rows),p95_evaluation_ms=float(np.percentile([r['evaluation_ms'] for r in rows],95)),
    active_capacity=1024,capacity_policy='fail closed; no FIFO/TTL deletion',
    original_initial_points_unchanged=True,unknown_points_per_call=5,rows=rows)
(A/'scale_report.json').write_text(json.dumps(freeze(report),indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
