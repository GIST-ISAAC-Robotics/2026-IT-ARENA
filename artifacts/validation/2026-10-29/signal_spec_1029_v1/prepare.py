"""10/29 제작측 신호등 설명을 반영한 팀 시험본. 공식 원본·기존 v2 시험본·production은 수정하지 않는다."""
import difflib
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
SOURCE=ROOT/'src/arena_gazebo/worlds/it_arena_official'
OFFICIAL=ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
COMMENT=BASE/'upstream/comment_6477451048.json'
# 제작측 수치(10/29 #3 댓글). 아래 SPEC 외의 값은 모두 팀 가정이며 manifest에 따로 적는다.
SPEC=dict(lamp_diameter_m=.015,rows=2,columns=5,module_bottom_m=.397,array_center_m=.419,
          face_to_floor_deg=60.,forward_of_lead_grid_line_m=.300)
PITCH_ACROSS=.030   # 10/9 댓글: 광원15 mm + 세트 사이15 mm
PITCH_UP=.020       # 10/9 댓글: 위아래 광원 사이5 mm
CAMERA_TRIAL=dict(xyz_m=[-.075,0.,.30],rpy_rad=[0.,0.,0.])  # 기존5랩 시험 카메라. 채택값 아님


def text(parent,tag,value):
    e=ET.SubElement(parent,tag);e.text=str(value);return e


def add(a,*terms):
    out=list(a)
    for scale,v in terms:
        out=[o+scale*x for o,x in zip(out,v)]
    return out


def rpy_from_axes(x,y,z):
    """열벡터 x,y,z로 이루어진 회전의 SDF roll/pitch/yaw."""
    r=[[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]]
    pitch=math.asin(max(-1.,min(1.,-r[2][0])))
    return [math.atan2(r[2][1],r[2][2]),pitch,math.atan2(r[1][0],r[0][0])]


def link_box(model,name,xyz,rpy,size,rgb='.05 .05 .05 1',collision=True):
    link=ET.SubElement(model,'link',name=name)
    text(link,'pose',' '.join(map(str,[*xyz,*rpy])))
    for kind in ('visual','collision') if collision else ('visual',):
        e=ET.SubElement(link,kind,name=name+'_'+kind)
        text(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size',' '.join(map(str,size)))
        if kind=='visual':
            mat=ET.SubElement(e,'material');text(mat,'ambient',rgb);text(mat,'diffuse',rgb)
    return link


def main():
    out=BASE/'fixture';out.mkdir(exist_ok=False)
    comment=json.loads(COMMENT.read_text(encoding='utf-8'))
    assert comment['id']==6477451048 and '397mm' in comment['body'] and '419mm' in comment['body'] and '60도' in comment['body']
    tree=ET.parse(SOURCE/'world.sdf');world=tree.find('world')
    scene=json.loads((SOURCE/'scene.json').read_text(encoding='utf-8'))
    model=world.find("model[@name='it_arena_track_static']")
    removed=[]
    for link in list(model.findall('link')):
        if link.get('name').startswith(('tl_','lamp_')):
            removed.append(link.get('name'));model.remove(link)
    assert 'lamp_red' in removed and 'lamp_green' in removed
    slots={s['index']:s for s in scene['starting_grid']['slots']}
    length=scene['starting_grid']['slot_length_m']
    # 선두 차량 = 진행 방향으로 가장 앞선 그리드(공식 배치에서 index2). 그리드라인 = 그 칸의 앞쪽 도색 경계(가정).
    lead=max(slots.values(),key=lambda s:s['s_m']);yaw=lead['yaw_rad']
    f=[math.cos(yaw),math.sin(yaw),0.];r=[math.sin(yaw),-math.cos(yaw),0.];up=[0.,0.,1.]
    other=next(s for s in slots.values() if s['row']==lead['row'] and s['col']!=lead['col'])
    # 가로 위치는 공개 도면2의 가로보 중앙 모듈과 공식 갠트리 형식을 따라 두 그리드 열의 가운데(도로 중앙 근처)로 가정한다.
    lateral=((other['x']-lead['x'])*r[0]+(other['y']-lead['y'])*r[1])/2
    line=add([lead['x'],lead['y'],0.],(length/2,f))
    center=add(line,(SPEC['forward_of_lead_grid_line_m'],f),(lateral,r));center[2]=SPEC['array_center_m']
    # 도면1(측면): 발광면은 바닥과60°, 위로 갈수록 그리드 쪽으로 나오며 법선은 그리드 쪽 아래를 향한다(해석).
    tilt=math.radians(SPEC['face_to_floor_deg'])
    n=[-math.sin(tilt)*f[0]-math.cos(tilt)*up[0],-math.sin(tilt)*f[1]-math.cos(tilt)*up[1],-math.cos(tilt)]
    u=[-math.cos(tilt)*f[0]+math.sin(tilt)*up[0],-math.cos(tilt)*f[1]+math.sin(tilt)*up[1],math.sin(tilt)]
    assert abs(sum(a*b for a,b in zip(n,u)))<1e-12 and n[2]<0
    # 하우징: 앞면이 발광 평면, 뒤로 20 mm. 하단이 정확히397 mm가 되도록 면 방향 높이를 정한다.
    depth=.020;half_up=(SPEC['array_center_m']-SPEC['module_bottom_m'])/math.sin(tilt)
    housing_center=add(center,(-depth/2,n))
    housing_rpy=rpy_from_axes(n,r,u)
    link_box(model,'trial_signal_housing',housing_center,housing_rpy,[depth,.16,2*half_up])
    corners=[add(housing_center,(a*depth/2,n),(b*.08,r),(c*half_up,u)) for a in (-1,1) for b in (-1,1) for c in (-1,1)]
    bottom=min(p[2] for p in corners);top=max(p[2] for p in corners)
    assert abs(bottom-SPEC['module_bottom_m'])<1e-9
    # 가로보·지지대는 팀 가정 형상. 기둥은 잔디/도로가 아니라 벽(도로 중심±.425–.475 m) 위에 둔다.
    beam_z=.455;beam_center=add(center,(.020,f));beam_center[2]=beam_z
    link_box(model,'trial_signal_beam',beam_center,[0,0,yaw],[.020,.95,.020])
    hanger_bottom=add(housing_center,(-depth/2,n),(half_up,u))
    hanger_center=[(hanger_bottom[i]+beam_center[i])/2 for i in range(3)]
    link_box(model,'trial_signal_hanger',hanger_center,[0,0,yaw],[.012,.012,max(.004,beam_z-hanger_bottom[2])])
    for side,name in ((-1,'left'),(1,'right')):
        post=add(beam_center,(side*.45,r));post[2]=(.40+beam_z+.01)/2
        link_box(model,f'trial_signal_wall_support_{name}',post,[0,0,yaw],[.020,.025,beam_z+.01-.40])
    lamps=[]
    lamp_rpy=rpy_from_axes(r,u,n)  # 원통 축(local z)=발광면 법선
    for column in range(SPEC['columns']):
        for row in range(SPEC['rows']):
            name=f'trial_lamp_{column}_{row}'
            xyz=add(center,((column-2)*PITCH_ACROSS,r),((.5-row)*PITCH_UP,u),(.001,n))
            lamps.append(dict(name=name,column=column,row=row,xyz_m=xyz,radius_m=SPEC['lamp_diameter_m']/2,normal=n,face_up=u))
            link=ET.SubElement(model,'link',name=name)
            text(link,'pose',' '.join(map(str,[*xyz,*lamp_rpy])))
            visual=ET.SubElement(link,'visual',name=name+'_visual')
            cyl=ET.SubElement(ET.SubElement(visual,'geometry'),'cylinder')
            text(cyl,'radius',SPEC['lamp_diameter_m']/2);text(cyl,'length','.002')
            mat=ET.SubElement(visual,'material')
            text(mat,'ambient','.004 .004 .004 1');text(mat,'diffuse','.004 .004 .004 1');text(mat,'emissive','0 0 0 1')
    assets={}
    for uri in sorted({e.text for e in tree.iter() if e.tag in ('albedo_map','uri') and e.text}):
        relative=Path(uri)
        if relative.is_absolute() or '..' in relative.parts or '://' in uri:raise ValueError('unexpected nonlocal asset: '+uri)
        asset=SOURCE/relative
        if not asset.is_file():raise FileNotFoundError(asset)
        target=out/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(asset,target);assets[uri]=hashlib.sha256(asset.read_bytes()).hexdigest()
    ET.indent(tree);tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    scene['traffic_light'].pop('lamp_poses',None)
    scene['traffic_light'].update(trial_layout='tilted_2x5_module_1029_comment',trial_array_center_m=center,
        trial_face_normal=n,trial_lamps=lamps,source_note='team derivative; official facility not installed/verified')
    with (out/'scene.json').open('w',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(scene,ensure_ascii=False,indent=2))
    (out/'centerline.csv').write_bytes((SOURCE/'centerline.csv').read_bytes())
    launch_source=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    original=launch_source.read_text(encoding='utf-8')
    old='    world_directory = gazebo_share / "worlds" / TRACK_DIRECTORIES[track]'
    assert original.count(old)==1
    changed=original.replace(old,'    world_directory = Path('+repr(str(out))+')  # registered test fixture only')
    old='        camera = profile["camera"]'
    assert changed.count(old)==1
    changed=changed.replace(old,old+'\n        camera["xyz_m"] = [-.075, 0., .30]  # trial fixed rear camera (existing 5-lap trial)\n        camera["rpy_rad"] = [0., 0., 0.]')
    with (out/'simulation.launch.py').open('w',encoding='utf-8',newline='\n') as stream:stream.write(changed)
    with (out/'simulation.diff').open('w',encoding='utf-8',newline='\n') as stream:
        stream.write(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
            fromfile='production/simulation.launch.py',tofile='fixture/simulation.launch.py')))
    manifest=dict(scope='team test fixture for the 10/29 organizer signal description; signal/camera only, no production change',
        organizer_values=SPEC,organizer_comment='https://github.com/MOSW626/istech-it-arena/issues/3#issuecomment-6477451048',
        lead_grid_index=lead['index'],lead_grid_line_xy=line[:2],array_center_xyz_m=center,face_normal=n,face_up=u,
        module_bottom_z_m=bottom,module_top_z_m=top,lateral_from_lead_slot_m=lateral,
        camera_xyz_m=CAMERA_TRIAL['xyz_m'],camera_rpy_rad=CAMERA_TRIAL['rpy_rad'],
        provisional=['lead grid line = front paint edge of the most forward official slot (index2)',
                     '300 mm measured horizontally to the lamp-array center along the lead slot heading',
                     'lateral center between the two grid columns (road center) from drawing 2 beam-centered module',
                     'drawing 1 read as true side view: face plane 60 deg to floor, normal toward grid and 30 deg downward',
                     'column pitch 30 mm and row pitch 20 mm from the 10/9 comment (15 mm gaps / 5 mm gap)',
                     'housing 160 mm wide x 20 mm deep; beam/hanger/wall-top supports are team shapes',
                     'official ArUco ID0/ID45 boards and grid are kept unchanged even if they conflict with the new signal',
                     'rear camera x=-.075 z=.30 horizontal is the existing trial view, not an adopted mount'],
        removed_links=removed,lamps=lamps,world_name=world.get('name'),assets_sha256=assets,
        sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (Path(__file__),COMMENT,launch_source,SOURCE/'world.sdf',SOURCE/'scene.json',SOURCE/'centerline.csv',OFFICIAL)})
    with (out/'manifest.json').open('w',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps(dict(center=center,normal=n,bottom=bottom,top=top,lead=lead['index'],lateral=lateral),ensure_ascii=False))


if __name__=='__main__':main()
