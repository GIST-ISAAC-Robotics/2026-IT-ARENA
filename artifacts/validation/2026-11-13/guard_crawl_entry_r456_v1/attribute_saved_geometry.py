"""평가용 정답/벽으로 반환의 실제 위치를 설명한다. 제어기/후보 입력으로 사용하지 않는다."""
import hashlib
import json
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

B=Path(__file__).resolve().parent
P=B.parent/'guard_crawl_drive_r452_v1/run1'
g=json.loads((B/'pinpoint_v1/approach_geometry.json').read_bytes())
t=g['identity']['reference_s']
lines=lambda p:[json.loads(s) for s in p.open()]
closest=lambda rows:min(rows,key=lambda r:abs(r[0]-t))
ego=closest(lines(P/'ego_poses.jsonl'))
actors={'lead':closest(lines(P/'actor_poses.jsonl'))}
actors.update({name:closest(rows) for name,rows in json.loads((P/'additional_actors.json').read_bytes())['poses'].items()})
boxes=[]
supports=[]
world=ET.parse(P/'runtime_world.sdf').getroot()
for model in world.iter('model'):
    if 'old610' in model.get('name','') and model.get('name','').endswith('_support'):
        supports.append(dict(name=model.get('name'),pose=list(map(float,model.findtext('pose').split()))))
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            name=collision.get('name','')
            size=collision.findtext('geometry/box/size')
            if 'wall' not in name or size is None:continue
            # Frozen track wall collision poses are expressed directly in world frame.
            assert model.findtext('pose') in (None,'0 0 0 0 0 0')
            assert link.findtext('pose') in (None,'0 0 0 0 0 0')
            boxes.append(dict(name=name,pose=list(map(float,collision.findtext('pose').split())),size=list(map(float,size.split()))))

def distance_to_boundary(point,box):
    x,y=point;bx,by,_,_,_,angle=box['pose'];length,width,_=box['size']
    dx,dy=x-bx,y-by
    qx=math.cos(angle)*dx+math.sin(angle)*dy
    qy=-math.sin(angle)*dx+math.cos(angle)*dy
    ox,oy=abs(qx)-length/2,abs(qy)-width/2
    distance=math.hypot(max(ox,0),max(oy,0)) if ox>0 or oy>0 else min(-ox,-oy)
    return distance

mesh=B.parent/'old_signal_610_r326_v1/support_frame_m.stl'
raw=mesh.read_bytes()
triangle_count=struct.unpack_from('<I',raw,80)[0]
assert len(raw)==84+50*triangle_count
triangles=[]
for i in range(triangle_count):
    values=struct.unpack_from('<12f',raw,84+50*i)[3:]
    triangles.append([values[j:j+3] for j in (0,3,6)])

def slices(height):
    segments=[]
    for tri in triangles:
        points=[]
        for a,b in zip(tri,tri[1:]+tri[:1]):
            if (a[2]-height)*(b[2]-height)<0:
                fraction=(height-a[2])/(b[2]-a[2])
                points.append([a[k]+fraction*(b[k]-a[k]) for k in (0,1)])
        if len(points)==2:segments.append(points)
    return segments

def segment_distance(point,segment):
    a,b=segment
    dx,dy=b[0]-a[0],b[1]-a[1]
    u=max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(point[0]-a[0]-u*dx,point[1]-a[1]-u*dy)

def support_distance(point,support,height):
    sx,sy,_,_,_,angle=support['pose']
    dx,dy=point[0]-sx,point[1]-sy
    local=(math.cos(angle)*dx+math.sin(angle)*dy,-math.sin(angle)*dx+math.cos(angle)*dy)
    return min(segment_distance(local,s) for s in slices(height))

points=[]
for x,y in g['sensor_threat_returns']['left']['points']:
    wx=ego[1]+math.cos(ego[3])*x-math.sin(ego[3])*y
    wy=ego[2]+math.sin(ego[3])*x+math.cos(ego[3])*y
    near=min(boxes,key=lambda b:distance_to_boundary((wx,wy),b))
    support=min(supports,key=lambda s:support_distance((wx,wy),s,.1346))
    points.append(dict(local_xy_m=[x,y],world_xy_m=[wx,wy],nearest_wall=near,
        wall_boundary_distance_m=distance_to_boundary((wx,wy),near),
        nearest_support=support,support_slice_boundary_distances_m={str(h):support_distance((wx,wy),support,h) for h in (.1,.1346,.2)},
        actor_center_distances_m={name:math.hypot(wx-a[1],wy-a[2]) for name,a in actors.items()}))
output=dict(scope='posthoc evaluation attribution only; truth/map never supplied to the controller or saved candidate',
    trial='two_actor_course_lap, two moving actors; not solo',reference_s=t,ego_pose=ego,
    ego_pose_offset_s=ego[0]-t,actor_poses=actors,
    ego_actor_center_distances_m={name:math.hypot(ego[1]-a[1],ego[2]-a[2]) for name,a in actors.items()},
    points=points,
    interpretation='Representative current left compact returns coincide with marker20 old610 support post cross-section, not static wall or nearby moving actor. Posthoc inference, not sensor-only identity proof; past seen history not all reclassified.',
    nominal_lidar_link_z_m=.1346,mesh_slice_heights_m=[.1,.1346,.2],
    height_scope='The planar poses omit roll/pitch/z. Multiple horizontal slices of the vertical support show the same boundary association; not an exact 3D ray/mesh hit proof.',
    previous_wall_only_hypothesis='rejected: nearest wall boundary is 0.181..0.188m away; first draft preserved in evaluation_attribution_wall_only_v1.json and superseded here',
    source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [P/'runtime_world.sdf',P/'ego_poses.jsonl',P/'actor_poses.jsonl',P/'additional_actors.json',mesh]})
(B/'pinpoint_v1/evaluation_attribution.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(ego_actor_center_distances_m=output['ego_actor_center_distances_m'],support_matches=[dict(name=p['nearest_support']['name'],distances_m=p['support_slice_boundary_distances_m']) for p in points]),ensure_ascii=False))
