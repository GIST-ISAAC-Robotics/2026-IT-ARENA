"""최신 2×5 광원, 한 격자의 정지 자세, 가상 상대 외형으로만 하는 평가. 제어 입력 아님."""
from itertools import product
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUN=HERE.parent/'solo_post_free_v1/run1'
WORLD=RUN/'runtime_world.sdf'
CAR=RUN/'runtime_vehicle.sdf'
GRID=ROOT/'src/arena_gazebo/worlds/it_arena_official/scene.json'
TILT=HERE.parent/'post_free_facility_v1/run1/runtime_vehicle.sdf'


def numbers(e,key,n):
    s=e.find(key)
    if s is None and key=='pose':
        return [0.]*n
    assert s is not None and not s.attrib
    v=list(map(float,s.text.split()));assert len(v)==n and all(map(math.isfinite,v))
    return v


def lamps():
    m=ET.parse(WORLD).find("world/model[@name='it_arena_track_static']")
    assert numbers(m,'pose',6)==[0.]*6
    result={}
    for item in m.findall('link'):
        name=item.get('name')
        if not name.startswith('lamp_'):
            continue
        pose=numbers(item,'pose',6);vis=item.find('visual')
        assert not any(pose[3:]) and not any(numbers(vis,'pose',6))
        radius,=numbers(vis,'geometry/sphere/radius',1)
        result[name]=(pose[:3],radius)
    assert len(result)==10
    return result


def camera(path):
    link=ET.parse(path).find("model/link[@name='b0183_link']")
    sens=link.find("sensor[@name='b0183_rgb']")
    assert not any(numbers(sens,'pose',6))
    x,y,z,r,p,yaw=numbers(link,'pose',6)
    assert y==r==yaw==0.
    return x,z,p,numbers(sens,'camera/horizontal_fov',1)[0],int(sens.findtext('camera/image/width')),int(sens.findtext('camera/image/height'))


def entry_height(origin, point, slot, length,width):
    """선분과 타 차량 수평 외형의 첫 교점 높이. 닫힌 외형, 광원은 카메라보다 높다."""
    c,s=math.cos(slot['yaw_rad']),math.sin(slot['yaw_rad'])
    x,y=origin[0]-slot['x'],origin[1]-slot['y']
    dx,dy=point[0]-origin[0],point[1]-origin[1]
    interval=[0.,1.]
    for start,delta,half in ((c*x+s*y,c*dx+s*dy,length/2),(-s*x+c*y,-s*dx+c*dy,width/2)):
        if abs(delta)<1e-12:
            if abs(start)>half:return None
        else:
            a,b=sorted(((-half-start)/delta,(half-start)/delta))
            interval=[max(interval[0],a),min(interval[1],b)]
            if interval[0]>interval[1]:return None
    assert point[2]>origin[2]
    return origin[2]+interval[0]*(point[2]-origin[2])


def inside(origin,point,yaw,cfg):
    dx,dy,dz=[a-b for a,b in zip(point,origin)]
    c,s=math.cos(yaw),math.sin(yaw)
    ahead=c*dx+s*dy;left=-s*dx+c*dy
    cp,sp=math.cos(cfg[2]),math.sin(cfg[2])
    depth=cp*ahead-sp*dz;up=sp*ahead+cp*dz
    f=cfg[4]/(2*math.tan(cfg[3]/2))
    return depth>0 and abs(f*left)<cfg[4]/2*depth and abs(f*up)<cfg[5]/2*depth


def case(grid,ego,cfg,floor_z,lights):
    yaw=ego['yaw_rad']
    origin=(ego['x']+cfg[0]*math.cos(yaw),ego['y']+cfg[0]*math.sin(yaw),floor_z+cfg[1])
    marks={}
    for name,(center,radius) in lights.items():
        cube=[tuple(v+k*radius for v,k in zip(center,ds)) for ds in product((-1,1),repeat=3)]
        center_hits=[];whole_hits=[]
        for other in grid:
            if other['index']==ego['index']:continue
            h=entry_height(origin,center,other,.20,.15)
            if h is not None:center_hits.append((h,other['index']))
            hs=[entry_height(origin,pt,other,.20,.15) for pt in cube]
            # 같은 상자 하나가 광원을 둘러싼 cube 전체의 시선을 가리는 충분조건.
            if all(u is not None for u in hs):whole_hits.append((max(hs),other['index']))
        marks[name]=dict(full_inside=all(inside(origin,q,yaw,cfg) for q in cube),
                         center_first=min(center_hits,default=None),
                         whole_sufficient=min(whole_hits,default=None))
    def level(keys,field,reducer):
        pairs=[marks[k][field] for k in keys]
        if reducer==max and any(v is None for v in pairs): return None
        return reducer((v[0] for v in pairs if v is not None),default=None)
    red=[k for k in marks if k.startswith('lamp_red')]
    green=[k for k in marks if k.startswith('lamp_green')]
    return dict(grid_ego=ego['index'], opponents=[q['index'] for q in grid if q!=ego],
                full_frame_lamps=sum(v['full_inside'] for v in marks.values()),
                first_red_center_height_m=level(red,'center_first',min),
                first_whole_red_sufficient_m=level(red,'whole_sufficient',min),
                all_red_whole_sufficient_m=level(red,'whole_sufficient',max),
                both_green_whole_sufficient_m=level(green,'whole_sufficient',max), lamps=marks)


def main():
    g=sorted(json.loads(GRID.read_bytes())['starting_grid']['slots'],key=lambda q:-q['s_m'])
    assert len(g)==6 and {s['index'] for s in g}==set(range(6))
    lights=lamps(); cams={'horizontal':camera(CAR),'up20':camera(TILT)}
    assert cams['horizontal'][:2]==cams['up20'][:2]
    results={}
    for label,cfg in cams.items():
        for dz in (0.,.01):
            for n in (2,3,5):
                active=g[:n]
                results[f'{label}_dz{dz}_n{n}']=[case(active,ego,cfg,dz,lights) for ego in active]
    d=dict(scope='평가 전용. 상대 20×15 cm 수직 상자, 명목 중심과 yaw, 동일 지면·정지. 실차 지붕/높이·배치 공차·검출 성능 미측정. null은 해당 충분조건 없음. 움직이는 앞차·빛번짐·가림 일부의 실제 인식률은 추정하지 않는다.',
           input_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (GRID,WORLD,CAR,TILT)},
           order=[s['index'] for s in g],camera=cams,results=results)
    with (HERE/'geometry.json').open('x',encoding='utf8') as f:
        json.dump(d,f,indent=2,ensure_ascii=False,allow_nan=False)
    for r in results['up20_dz0.0_n5']:
        print(r['grid_ego'],r['full_frame_lamps'],r['first_whole_red_sufficient_m'],
              r['all_red_whole_sufficient_m'],r['both_green_whole_sufficient_m'])


if __name__=='__main__':
    main()
