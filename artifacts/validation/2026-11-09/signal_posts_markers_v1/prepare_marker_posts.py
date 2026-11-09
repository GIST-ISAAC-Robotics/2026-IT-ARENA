"""설치 사실은 확정, 형상·위치는 임시인 마커 지주 시험 SDF.

ID20의 판은 코너 위라 같은 횡단면 남쪽 지주가 도로를 막는다. 판은 옮기지
않고 지지 틀을 접근 방향으로 372.5 mm 이동해 잔디 위에 둔다. 실물 설계 아님.
"""
import hashlib,json,math,sys
import xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path.insert(0,str(R/'scripts'))
from surface_contact_eval import SurfaceMap,rectangle
W=R/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
surface=SurfaceMap.from_world(W)
root=ET.Element('sdf',version='1.9');rows=[]

def shape(model,name,x,y,z,yaw,length,width,height):
    link=ET.SubElement(model,'link',name=name)
    for kind in ('visual','collision'):
        element=ET.SubElement(link,kind,name=name+'_'+kind)
        ET.SubElement(element,'pose').text=f'{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.9f}'
        ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text=f'{length} {width} {height}'
        if kind=='visual':
            material=ET.SubElement(element,'material')
            ET.SubElement(material,'ambient').text='0.3 0.3 0.3 1'
            ET.SubElement(material,'diffuse').text='0.45 0.45 0.45 1'

for marker,x,y,yaw,upstream in [(20,6.155,6.,0.,.3725),(30,2.48,3.505,math.pi/2,0.)]:
    model=ET.SubElement(root,'model',name=f'eval_marker_{marker}_supports')
    ET.SubElement(model,'static').text='true'
    gx,gy=x+upstream*math.cos(yaw),y+upstream*math.sin(yaw)
    for sign in (-1,1):
        cx,cy=gx-sign*.3725*math.sin(yaw),gy+sign*.3725*math.cos(yaw)
        for part,length,width,height,z in [('post',.01462,.015,.455,.2375),('foot',.05,.04,.01,.005)]:
            name=f'marker_{marker}_{part}_{"left" if sign==1 else "right"}'
            shape(model,name,cx,cy,z,yaw,length,width,height)
            poly=rectangle(cx,cy,yaw,length,width)
            road=poly.intersection(surface.road).area;wall=poly.intersection(surface.walls).area
            assert road<1e-10 and wall<1e-10 and surface.point_class(cx,cy)=='grass',(name,road,wall)
            rows.append(dict(name=name,id=marker,part=part,center=[cx,cy],yaw=yaw,length=length,width=width,
                bottom=z-height/2,top=z+height/2,road_overlap_m2=road,wall_overlap_m2=wall))
    shape(model,f'marker_{marker}_beam',gx,gy,.4575,yaw,.01462,.760,.015)
    if upstream:
        shape(model,f'marker_{marker}_arm',(gx+x)/2,(gy+y)/2,.4575,yaw,upstream,.015,.015)
    # 판 상단0.45m와 상부0.45m를 짧은 클립으로 잇는 임시 표현. 마커 원본 불변.
    shape(model,f'marker_{marker}_clip',x+.003*math.cos(yaw),y+.003*math.sin(yaw),.445,yaw,.006,.015,.020)
path=A/'marker_posts_provisional.sdf'
ET.indent(root);ET.ElementTree(root).write(path,encoding='utf8',xml_declaration=True)
report=dict(scope='user-confirmed presence; provisional gantry count, dimensions and poses; no controller truth input',
    section_source='signal_post_source_v1 STL post/foot section; not a marker support drawing',
    source_world_sha256=hashlib.sha256(W.read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    marker_board_poses_unchanged=True,id20_support_upstream_shift_m=.3725,
    id20_reason='same-plane south post lies on corner exit road; retain board and support from upstream grass',links=rows)
(A/'marker_posts_provisional.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(models=2,low_parts=len(rows),all_on_grass=True,fixture_sha256=report['fixture_sha256'])))
