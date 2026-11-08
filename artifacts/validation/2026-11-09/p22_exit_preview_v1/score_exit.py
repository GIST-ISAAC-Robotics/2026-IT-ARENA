"""제안을 고정한 뒤 실제 월드에서 정적 경로를 대조한다. 실행/제어 입력 아님."""
import bisect,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from surface_contact_eval import SurfaceMap,VehicleGeometry,evaluate_pose,steering_footprint

A=Path(__file__).resolve().parent;D=A.parent/'p22_entry_wiring_v1/run1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    source=A/'exit_paths.json';before=sha(source);raw=json.loads(source.read_bytes())
    poses=[json.loads(l) for l in (D/'ego_poses.jsonl').open()];times=[p[0] for p in poses]
    surface=SurfaceMap.from_world(D/'runtime_world.sdf');g=VehicleGeometry();rows=[]
    for row in raw['rows']:
        if len(row['paths'])!=1:continue
        pth=row['paths'][0];pose=poses[min(range(max(0,bisect.bisect_left(times,row['t'])-1),min(len(poses),bisect.bisect_left(times,row['t'])+2)),key=lambda i:abs(times[i]-row['t']))]
        assert abs(pose[0]-row['t'])<.01
        c,s=math.cos(pose[3]),math.sin(pose[3]);off=0;gap=math.inf;other=False;firstoff=None
        for rx,ry,h,steer,dist in pth['poses']:
            cx=rx+g.wheelbase_m/2*math.cos(h);cy=ry+g.wheelbase_m/2*math.sin(h)
            x,y=pose[1]+c*cx-s*cy,pose[2]+s*cx+c*cy;heading=pose[3]+h
            fp=steering_footprint(x,y,heading,steer,g);gap=min(gap,fp.distance(surface.walls))
            e=evaluate_pose(surface,x,y,heading,steer,g)
            off=max(off,e['off_counts']['any'])
            if e['off_counts']['any']>2 and firstoff is None:firstoff=dist
            other|=any(w['fractions']['wall']>1e-9 or w['fractions']['other']>1e-9 for w in e['wheels'].values())
        rows.append(dict(t=row['t'],speed=row['speed'],first_command=pth.get('first_output',[None])[0],
                         max_offroad_wheels=off,min_wall_gap_m=gap,wall_or_other=other,
                         first_three_wheels_s_m=firstoff,end_body_world=[x,y,heading],
                         sampled_geometry_passed=off<=2 and gap>0 and not other))
    assert sha(source)==before
    out=dict(scope='fixed sensor path evaluated statically; no new driving',input_sha256=before,rows=rows)
    (A/'exit_geometry.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
    for r in rows:print(r)
if __name__=='__main__':main()
