"""Evaluation-only schedule geometry against a saved ego trace; no controller input."""
from pathlib import Path
import copy,json,math
import numpy as np
from validate_g1_traffic import load_centerline,load_config,actor_start_s,check_route
from traffic_corner_route import corner_pose,corner_geometry
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons,rectangle

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
original=A.parent/'retained_corner_agreement_r46_v1'
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=load_config(original/'new_track_course.json')
line,scene=load_centerline(world)
report=json.loads((original/'run1/report.json').read_bytes())
begin=report['case_window']['green_sim_s'];end=report['case_window']['end_sim_s']
trace=[q for q in json.loads((original/'run1/trace.json').read_bytes()) if begin<=q['t']<=end]
road,obstacles=road_polygon(world/'world.sdf'),obstacle_polygons(world/'world.sdf')
results=[]
for start in (-.15,-.45,-.65,-.85):
    c=copy.deepcopy(cfg);case=c['cases'][0];case['actor_start_center_gap_m']=start
    # New phase candidates have longer travel to the same exit; duration is geometry-derived,
    # not a change to r46's failure criteria or report.
    s0=actor_start_s(c,scene,0,case);g=corner_geometry(c,line,case,s0)
    duration=max(32.,(g['straight']+g['arc']+.4)/.35)
    case['duration_s']=duration;case['schedule']=[dict(from_s=0.,to_s=duration,speed_mps=.35)]
    route=check_route(c,line,case,s0,road,obstacles)
    slot=next(q for q in scene['starting_grid']['slots'] if q['index']==c['launch']['grid_slot'])
    initial=rectangle(*corner_pose(c,line,s0,case,0),.2,.15).distance(rectangle(slot['x'],slot['y'],slot['yaw_rad'],.2,.15))
    longest=current=0.;previous=None;minimum=math.inf;selected=[]
    for q in trace:
        x,y,yaw=corner_pose(c,line,s0,case,q['t']-begin)
        dx,dy=x-q['x'],y-q['y'];co,si=math.cos(q['yaw']),math.sin(q['yaw'])
        longitudinal,lateral=co*dx+si*dy,-si*dx+co*dy
        angle=math.degrees(math.atan2(math.sin(q['yaw']-math.pi/2),math.cos(q['yaw']-math.pi/2)))
        diff=math.degrees(abs(math.atan2(math.sin(yaw-q['yaw']),math.cos(yaw-q['yaw']))))
        paired=20<=angle<=70 and abs(longitudinal)<=.30 and .15<=abs(lateral)<=.55 and diff<=30
        gap=rectangle(x,y,yaw,.2,.15).distance(rectangle(q['x'],q['y'],q['yaw'],.2,.15));minimum=min(minimum,gap)
        if previous is not None:
            dt=q['t']-previous[0];current=current+dt if 0<dt<=.2 and paired and previous[1] else 0
            longest=max(longest,current)
        previous=(q['t'],paired)
        if paired:selected.append(dict(t=q['t'],longitudinal_m=longitudinal,lateral_m=lateral,heading_difference_deg=diff))
    results.append(dict(start_gap_m=start,duration_to_exit_plus_40cm_s=duration,route=route,
        initial_gap_m=initial,minimum_against_old_ego_m=minimum,paired_duration_against_old_ego_s=longest,paired_rows=selected))
out=dict(scope='counterfactual actor phases against unchanged recorded ego, not closed-loop evidence',
    source_trace=str((original/'run1/trace.json').relative_to(ROOT)),results=results)
(A/'comparison.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps([{k:v for k,v in r.items() if k not in ('route','paired_rows')} for r in results]))
