"""제안 뒤에만 실제 노면·벽과 대조한다. 정답은 경로 제안에 역주입하지 않는다."""
import json,math,hashlib,bisect
from pathlib import Path
import numpy as np
from dataclasses import astuple
from extend_path import retained_poses
from surface_contact_eval import SurfaceMap,VehicleGeometry,evaluate_pose,steering_footprint

A=Path(__file__).resolve().parent;D=A.parent/'p22_route_connected_v1/run1'
source=A/'probe_summary.json';digest=hashlib.sha256(source.read_bytes()).hexdigest()
raw=json.loads(source.read_bytes());poses=[json.loads(l) for l in (D/'ego_poses.jsonl').open()]
times=[p[0] for p in poses];surface=SurfaceMap.from_world(D/'runtime_world.sdf');g=VehicleGeometry();rows=[]
selected=raw['new_first_exit']
assert selected is not None
paths=[dict(t=selected['t'],candidate=dict(poses=[astuple(p) for p in retained_poses(selected['path'])]))]
for row in paths:
    idx=bisect.bisect_left(times,row['t']);p=min(poses[max(0,idx-1):idx+2],key=lambda v:abs(v[0]-row['t']))
    assert abs(p[0]-row['t'])<.01
    c,s=math.cos(p[3]),math.sin(p[3]);off=0;gap=math.inf;other=0;count=0
    path=row['candidate']['poses'];nearest=int(np.argmin([math.hypot(v[0]+g.wheelbase_m/2*math.cos(v[2]),v[1]+g.wheelbase_m/2*math.sin(v[2])) for v in path]))
    for rx,ry,h,steer,dist in path[nearest:]:
        cx,cy=rx+g.wheelbase_m/2*math.cos(h),ry+g.wheelbase_m/2*math.sin(h)
        x,y=p[1]+c*cx-s*cy,p[2]+s*cx+c*cy;heading=p[3]+h
        footprint=steering_footprint(x,y,heading,steer,g);gap=min(gap,footprint.distance(surface.walls))
        e=evaluate_pose(surface,x,y,heading,steer,g);off=max(off,e['off_counts']['any']);count+=1
        other+=int(any(w['fractions']['wall']>1e-9 or w['fractions']['other']>1e-9 for w in e['wheels'].values()))
    rows.append(dict(t=row['t'],path_samples=count,max_offroad_wheels=off,min_wall_gap_m=gap,wall_other_samples=other,
                     sampled_geometry_passed=off<=2 and gap>0 and other==0))
out=dict(scope='static geometry of selected aligned body path; wheel steering estimated from its tangent, no new driving',input_sha256=digest,
         input_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==digest,rows=rows)
(A/'geometry_summary.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
