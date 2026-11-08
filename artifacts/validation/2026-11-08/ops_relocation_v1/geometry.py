"""이동 가능한 심판 배치 시험 지점의 기하와 과거 정지 창을 실행 전 대조한다."""
import csv,hashlib,json,math,sys
from pathlib import Path
from shapely.geometry import LineString
from g1_traffic_eval import rectangle
from audit_local_pursuit_road import road_polygon
from settle import hold_window
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
road=road_polygon(world/'world.sdf')
line=list(csv.DictReader((world/'centerline.csv').open()))
footprints=[]
for row in line:
    s,x,y=map(float,(row['s_m'],row['x_m'],row['y_m']))
    if 12.-1e-8<=s<=13.8+1e-8:footprints.append((s,rectangle(x,y,-math.pi/2,.20,.15)))
assert len(footprints)>150 and max(p.difference(road).area for _,p in footprints)<=1e-6
old=A.parent/'ops_newcourse_v1'
report=json.loads((old/'run1/report.json').read_bytes());rows=[json.loads(l) for l in (old/'run1/ego_poses.jsonl').open()]
result=[]
for h in report['steps']:
    if h['step'] not in ('first_stop_hold','final_stop_hold'):continue
    a,b=h['from_sim_s'],h['to_sim_s'];w=[p for p in rows if a<=p[0]<=b]
    speeds=[(y[0]-a,math.dist(x[1:3],y[1:3])/(y[0]-x[0])) for x,y in zip(w,w[1:])]
    viol=[(t,v) for t,v in speeds if v>.05]
    assert viol
    result.append(dict(step=h['step'],original_window_peak_mps=max(v for _,v in speeds),
        last_above_threshold_relative_s=viol[-1][0],peak_relative_s=max(speeds,key=lambda a:a[1])[0],
        bins_100ms_max_mps=[max(v for t,v in speeds if .1*k<t<=.1*(k+1)) for k in range(5)]))
r=dict(scope='evaluation-only geometry and old stored STOP signal windows; not an actual restart',
    tested_s=[footprints[0][0],footprints[-1][0]],poses=len(footprints),body_outside_max_m2=max(p.difference(road).area for _,p in footprints),
    minimum_road_edge_m=min(p.distance(road.boundary) for _,p in footprints),prior_windows=result)
(A/'preflight_review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print(r)
