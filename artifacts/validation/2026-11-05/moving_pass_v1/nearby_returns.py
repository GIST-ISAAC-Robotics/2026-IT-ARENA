"""영속 정지 이전/중 실제 판단의 간격=0 광선을 찾는다. 제어기에 평가지식을 전달하지 않는다."""
from pathlib import Path
import json
import math
import sys
from collections import Counter
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
from arena_autonomy.occupancy_timing import per_ray_padding
from review import lines,read

HERE=Path(__file__).resolve().parent
def violations(points,pads,steer,cfg):
    k=math.tan(steer)/cfg.wheelbase_m
    hits=set()
    for d in (0.,.04):
        yaw=k*d;c,s=math.cos(yaw),math.sin(yaw)
        rx,ry=(d-cfg.wheelbase_m/2,0.) if abs(k)<1e-5 else (s/k-cfg.wheelbase_m/2,(1-c)/k)
        dx,dy=points[:,0]-rx,points[:,1]-ry
        hit=(abs(c*dx+s*dy-cfg.wheelbase_m/2)<=cfg.length_m/2+pads)&(abs(-s*dx+c*dy)<=cfg.width_m/2+pads)
        hits.update(np.flatnonzero(hit).tolist())
    return hits

def main(name):
    root=HERE/name;r=root/'run1';dest=root/'nearby_returns.json'
    if dest.exists():raise FileExistsError(dest)
    events=lines(r/'follow_decisions.jsonl');head=next(events)['data']
    cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
    green=read(r/'report.json')['case_window']['green_sim_s'];rows=[];counts=Counter()
    for e in events:
        if e['kind']!='call':continue
        d=e['data'];speed,steer,detail=d['output'];entry=detail.get('moving_pass',{}).get('entry',{})
        rows_zero=[q for q in entry.get('candidates',[]) if q.get('reason')=='known_occupied_command_hold' and q.get('command_clearance_m')==0.]
        if not rows_zero:continue
        inp,scan,hist=restore_call(d['input'],motion)
        sc=motion_scene(scan,hist,inp['reference_s'],timing_verified=inp['timing_verified'],epoch=inp['epoch'],
                        rear_half_angle_deg=inp['rear_half_angle_deg'])
        rays=[r for r in sc.rays if r.status=='return'];pts=np.array([r.hit_xy_m for r in rays])
        for candidate in rows_zero:
            side=candidate['current_outer_wall'];walls=entry['reference']['walls'];wall=walls[side]
            pads=per_ray_padding(sc,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
            if 'outer_boundary_padding' in entry or 'outer_boundary_padding' in candidate:
                pads,_=current_outer_wall_padding(sc,cfg,side,wall)
            hits=violations(pts,pads,inp['previous_steering_rad'],cfg)|violations(pts,pads,candidate['steering_rad'],cfg)
            ids=[rays[i].source_index for i in sorted(hits)]
            xs=[pts[i,0] for i in hits];ys=[pts[i,1] for i in hits]
            label=('prior_axle_region' if xs and max(xs)<0 else 'front_or_mixed') if ids else 'unexplained'
            counts[label]+=1
            rows.append(dict(t=round(sc.t_ref_s-green,3),rays=ids,label=label,
                x=[min(xs),max(xs)] if xs else [],y=[min(ys),max(ys)] if ys else [],
                padding=[float(min(pads[list(hits)])),float(max(pads[list(hits)]))] if hits else []))
    result=dict(scope='current_or_first_4cm_occupied_returns_on_recorded_commands_sensor_frame_only',
        time_relative_to_green_s=True,counts=counts,rows=rows)
    with dest.open('x',encoding='utf-8') as f:json.dump(result,f,indent=1);f.write('\n')
    print(counts,rows[:1])
if __name__=='__main__':main(sys.argv[1])
