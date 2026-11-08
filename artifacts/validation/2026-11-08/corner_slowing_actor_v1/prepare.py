"""New evaluation scenario: fixed gradual speed schedule, no controller changes."""
from pathlib import Path
import copy,json,math
import numpy as np
from validate_g1_traffic import load_config,load_centerline,actor_start_s,check_route
from traffic_corner_route import corner_geometry,corner_pose
from g1_traffic_eval import actor_travel_m
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons,rectangle

A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'current_corner_fallback_r47_v1'
assert not (A/'new_track_course.json').exists()
cfg=load_config(OLD/'new_track_course.json');case=cfg['cases'][0]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
line,scene=load_centerline(world);s0=actor_start_s(cfg,scene,0,case);g=corner_geometry(cfg,line,case,s0)
schedule=[];clock=0.
def stage(duration,speed):
    global clock
    schedule.append(dict(from_s=clock,to_s=clock+duration,speed_mps=speed));clock+=duration
# Four 0.1s steps approximate 0.5m/s^2 deceleration over 0.1m before the arc.
stage((g['straight']-.1)/.35,.35)
for speed in (.325,.275,.225,.175):stage(.1,speed)
stage(g['arc']/.15,.15)
for speed in (.175,.225,.275,.325):stage(.1,speed)
stage(.3/.35,.35)
case['schedule']=schedule;case['duration_s']=clock
case['expected']='오른쪽 상대가 직선0.35m/s에서 코너0.15m/s로 감속한다. 실제 코너 병렬/출구를 측정하며 기존32초 실패와 별개다.'
cfg['scope']='현재 r47 제어기 불변. 코너 감속 상대의 고정 평가 각본으로 실제 병렬 관계를 확인한다.'
route=check_route(cfg,line,case,s0,road_polygon(world/'world.sdf'),obstacle_polygons(world/'world.sdf'))
assert math.isclose(actor_travel_m(schedule,clock),g['straight']+g['arc']+.4,abs_tol=1e-9)
slot=next(q for q in scene['starting_grid']['slots'] if q['index']==cfg['launch']['grid_slot'])
initial=rectangle(*corner_pose(cfg,line,s0,case,0),.2,.15).distance(rectangle(slot['x'],slot['y'],slot['yaw_rad'],.2,.15))
assert initial>=.08
joins=[]
for t in [s['to_s'] for s in schedule[:-1]]:
    a,b=corner_pose(cfg,line,s0,case,t-1e-7),corner_pose(cfg,line,s0,case,t+1e-7)
    joins.append(dict(t=t,position_step_m=math.dist(a[:2],b[:2]),yaw_step_rad=abs(a[2]-b[2])))
assert all(j['position_step_m']<1e-6 and j['yaw_step_rad']<1e-6 for j in joins)
geometry=dict(initial_ego_actor_gap_m=initial,route=route,scheduled_duration_s=clock,
    slowdown_start_s=schedule[0]['to_s'],arc_begin_s=schedule[4]['to_s'],arc_end_s=schedule[5]['to_s'],
    speed_step_mps=.05,speed_step_interval_s=.1,joins=joins,
    scope='kinematic fixed schedule; no evaluation truth given to ego controller')
(A/'new_track_course.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
(A/'geometry.json').write_text(json.dumps(geometry,indent=2)+'\n',encoding='utf8')
for name in ('check_runtime.py','assess.py','evaluation_source.json','run.py','review.py'):
    (A/name).write_bytes((OLD/name).read_bytes())
s=(OLD/'run.sh').read_text(encoding='utf8').replace(OLD.name,A.name)
(A/'run.sh').write_text(s,encoding='utf8',newline='\n')
print(json.dumps(dict(duration_s=clock,initial_gap_m=initial,route=route,join_count=len(joins))))
