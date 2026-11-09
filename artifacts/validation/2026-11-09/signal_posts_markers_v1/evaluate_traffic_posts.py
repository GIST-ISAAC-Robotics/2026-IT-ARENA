"""실제 실행 월드의 신호 지주와 고밀도 자차 자세를 대조하는 사후 평가."""
import json,sys,math,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path.insert(0,str(R/'scripts'))
from build_experimental_track import rectangle
run=Path(sys.argv[1]).resolve()
track=ET.parse(run/'runtime_world.sdf').find("./world/model[@name='it_arena_track_static']")
parts=[]
for link in track.findall('link'):
    name=link.get('name','')
    if not name.startswith(('tl_post_','tl_foot_')):continue
    assert link.find('pose') is None
    collision=link.find('collision');x,y,z,r,p,yaw=map(float,collision.findtext('pose').split())
    dx,dy,dz=map(float,collision.findtext('geometry/box/size').split())
    assert r==p==0
    parts.append((name,rectangle(x,y,yaw,dx,dy)))
assert len(parts)==4
result={name:dict(minimum_m=math.inf,overlap_samples=0,closest=None) for name,_ in parts}
poses=[json.loads(s) for s in (run/'ego_poses.jsonl').read_text().splitlines()]
report=json.loads((run/'report.json').read_text());count=0
for t,x,y,yaw in poses:
    body=rectangle(x,y,yaw,.2,.15);count+=1
    for name,poly in parts:
        gap=body.distance(poly);row=result[name]
        if gap<row['minimum_m']:row['minimum_m']=gap;row['closest']=dict(t=t,x=x,y=y,yaw=yaw)
        row['overlap_samples']+=int(body.intersection(poly).area>1e-9)
out=dict(scope='dense sampled body 2D footprint against actual runtime signal posts/feet, not physics continuous-contact proof',
    samples=count,parts=result,drive_and_stop_recorded=True,
    reached_signal_y=max(p[2] for p in poses)>=6.145,
    source_unchanged=report.get('source_unchanged'),shutdown_clean=report.get('shutdown_clean'))
(run/'dense_post_clearance.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=1))
