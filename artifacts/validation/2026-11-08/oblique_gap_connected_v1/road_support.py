"""차체 평가 자세와 정적 SDF 박스의 기하 대조. 접촉력 측정이 아니다."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
world=HERE/'run1/runtime_world.sdf'
root=ET.parse(world).getroot()
pose=lambda n:list(map(float,(n.findtext('pose') or '0 0 0 0 0 0').split()))
boxes=[]
for model in root.findall('./world/model'):
    assert pose(model)==[0.]*6, 'nontrivial model transform not supported'
    for link in model.findall('link'):
        lp=pose(link)
        for col in link.findall('collision'):
            cp=pose(col); node=col.find('geometry/box/size')
            if node is None: continue
            sx,sy,sz=map(float,node.text.split())
            top=lp[2]+cp[2]+sz/2
            if not -.01<top<.02: continue
            assert abs(lp[3])+abs(lp[4])+abs(cp[3])+abs(cp[4])<1e-7, 'tilted floor not supported'
            x=lp[0]+cp[0]*math.cos(lp[5])-cp[1]*math.sin(lp[5])
            y=lp[1]+cp[0]*math.sin(lp[5])+cp[1]*math.cos(lp[5])
            boxes.append((col.attrib['name'],x,y,top,sx,sy,lp[5]+cp[5]))
trace=json.loads((HERE/'run1/trace.json').read_text())
samples=[]
for target in (33.,33.8,34.1,34.5,34.8):
    q=min(trace,key=lambda r:abs(r['progress_m']-target)); yaw=q['yaw']
    sides={}
    for side,by in [('left',.0675),('right',-.0675)]:
        x=q['x']-.0675*math.cos(yaw)-by*math.sin(yaw)
        y=q['y']-.0675*math.sin(yaw)+by*math.cos(yaw)
        hits=[]
        for name,cx,cy,top,sx,sy,angle in boxes:
            dx,dy=x-cx,y-cy
            u=dx*math.cos(angle)+dy*math.sin(angle)
            v=-dx*math.sin(angle)+dy*math.cos(angle)
            if abs(u)<=sx/2+.006 and abs(v)<=sy/2+.006 and -.01<top<.02:
                hits.append(dict(collision=name,top_m=top))
        sides[side]=dict(wheel_center_xy=[x,y],overlap_boxes=hits)
    samples.append(dict(progress_m=q['progress_m'],time_s=q['time'],rear=sides))
result=dict(scope='static geometry only, no actual contact forces or suspension proof',
            world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(), samples=samples)
(HERE/'road_support.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('saved road_support.json')
