"""138 활성점의 유한 대표 비용. Windows 단독 Python이며 장치 처리율 증거가 아니다."""
from pathlib import Path
import json,sys,time
import numpy as np
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface'),str(A/'stage/tests'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
from test_guard_memory_command import seeded,PARTIAL
from test_independent_guard_route import scan
from arena_autonomy.guard_memory_command import limited_memory_command
from arena_autonomy.scan_input import project_scan

old=np.column_stack((np.linspace(.38,.42,138),-.015+.004*np.sin(np.arange(138))))
p,h=seeded(old);rows=[]
for tick in range(100):
    now=.6+tick*.02;raw=scan(.5+(tick//5)*.1,[2.]*500)
    raw.ranges[250]=raw.ranges[375]=float('nan')
    points,_,_=project_scan(raw,h,now,timing_verified=True)
    start=time.perf_counter()
    result=limited_memory_command(p,PARTIAL,raw,points,h,now,.8,0.,0.,0.,.135,.20,.15)
    elapsed=(time.perf_counter()-start)*1000
    assert result['accepted'] and result['speed_cap_mps']==.1 and result['retained_points']==138
    report=result['current_ray_clearance']
    rows.append(dict(now_s=now,evaluation_ms=elapsed,
        fresh=report['reason']!='same_scan_no_new_ray_evidence',centers=report.get('supported_centers')))

resolved=[]
for _ in range(5):
    p,h=seeded(old);raw=scan(.5,[2.]*500);raw.ranges[375]=float('nan')
    points,_,_=project_scan(raw,h,.6,timing_verified=True)
    start=time.perf_counter()
    result=limited_memory_command(p,PARTIAL,raw,points,h,.6,.8,0.,0.,0.,.135,.20,.15)
    elapsed=(time.perf_counter()-start)*1000
    assert result['accepted'] and result['speed_cap_mps']==.8 and result['active_points_ray_cleared']==138
    resolved.append(elapsed)

def summary(values):return dict(calls=len(values),maximum_ms=max(values),p95_ms=float(np.percentile(values,95)))
report=dict(scope='synthetic 138 retained points; 53 surrounding samples; Windows Python fallback; projection excluded; no actual r531 replay or ROS/Jetson',
    fresh_partial_support=summary([r['evaluation_ms'] for r in rows if r['fresh']]),
    repeated_same_scan=summary([r['evaluation_ms'] for r in rows if not r['fresh']]),
    all_138_cleared=summary(resolved),maximum_active=138,rows=rows)
(A/'cost_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
