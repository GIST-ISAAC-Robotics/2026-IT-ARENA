"""공식 가로 배열의 저속 출발 시험본. 현행 production/공식 원본은 수정하지 않는다."""
from copy import deepcopy
import difflib
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]


def text(parent,tag,value):
    e=ET.SubElement(parent,tag);e.text=str(value);return e


def box(model,name,xyz,size,yaw):
    link=ET.SubElement(model,'link',name=name)
    text(link,'pose',' '.join(map(str,[*xyz,0,0,yaw])))
    for kind in ('visual','collision'):
        e=ET.SubElement(link,kind,name=name+'_'+kind)
        shape=ET.SubElement(ET.SubElement(e,'geometry'),'box');text(shape,'size',' '.join(map(str,size)))
        if kind=='visual':
            mat=ET.SubElement(e,'material');text(mat,'ambient','.05 .05 .05 1');text(mat,'diffuse','.05 .05 .05 1')


def main():
    out=BASE/'fixture';out.mkdir(exist_ok=False)
    source=ROOT/'src/arena_gazebo/worlds/it_arena_official'
    tree=ET.parse(source/'world.sdf');world=tree.find('world')
    scene=json.loads((source/'scene.json').read_text(encoding='utf-8'))
    model=world.find("model[@name='it_arena_track_static']")
    removed=[]
    for link in list(model.findall('link')):
        name=link.get('name')
        if name.startswith(('tl_','lamp_')):
            removed.append(name);model.remove(link)
    assert 'lamp_red' in removed and 'lamp_green' in removed
    # 공식 좌우 순서/15mm 광원/쌍간 여백 해석. 중심40cm·판/지지 형상은 시험 가정.
    official=ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
    with zipfile.ZipFile(official) as archive:
        raw_scene=json.loads(archive.read(next(n for n in archive.namelist() if n.endswith('scene.json'))))
    spec=raw_scene['traffic_light']['pose'];x,y,yaw=spec['x'],spec['y'],spec['yaw_rad']
    scene['traffic_light']['pose']=deepcopy(spec)
    f=(math.cos(yaw),math.sin(yaw));right=(math.sin(yaw),-math.cos(yaw))
    def point(along,across,z):return [x+along*f[0]+across*right[0],y+along*f[1]+across*right[1],z]
    box(model,'trial_signal_panel',point(0,0,.4),[.012,.16,.06],yaw)
    box(model,'trial_signal_beam',point(0,-.10,.455),[.025,.40,.025],yaw)
    box(model,'trial_signal_hanger',point(0,0,.43875),[.012,.012,.0175],yaw)
    # 벽 위 지지 가정이며 C1 높이의 도로에 새 기둥을 세우지 않는다.
    box(model,'trial_signal_wall_support',point(0,-.275,.4025),[.025,.025,.105],yaw)
    lamps=[]
    for column in range(5):
        for row in range(2):
            name=f'trial_lamp_{column}_{row}';lamps.append(dict(name=name,column=column,row=row))
            link=ET.SubElement(model,'link',name=name)
            xyz=point(-.014,(column-2)*.03,.4+(.5-row)*.02)
            text(link,'pose',' '.join(map(str,[*xyz,0,0,0])))
            visual=ET.SubElement(link,'visual',name=name+'_visual')
            text(ET.SubElement(ET.SubElement(visual,'geometry'),'sphere'),'radius','.0075')
            mat=ET.SubElement(visual,'material')
            text(mat,'ambient','.004 .004 .004 1');text(mat,'diffuse','.004 .004 .004 1');text(mat,'emissive','0 0 0 1')
    ET.indent(tree);tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    scene['traffic_light']['trial_layout']='horizontal_four_red_plus_green_paired_center_40cm'
    (out/'scene.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'centerline.csv').write_bytes((source/'centerline.csv').read_bytes())
    launch_source=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    original=launch_source.read_text(encoding='utf-8')
    old='    world_directory = gazebo_share / "worlds" / TRACK_DIRECTORIES[track]'
    assert original.count(old)==1
    changed=original.replace(old,'    world_directory = Path('+repr(str(out))+')  # registered test fixture only')
    old='        camera = profile["camera"]'
    assert changed.count(old)==1
    changed=changed.replace(old,old+'\n        camera["xyz_m"] = [-.075, 0., .30]  # trial fixed rear camera\n        camera["rpy_rad"] = [0., 0., 0.]')
    # 노드·구동/보호·타임아웃 코드의 변경은 없다. 아래 두 설정 대체만 검토한다.
    (out/'simulation.launch.py').write_text(changed,encoding='utf-8')
    (out/'simulation.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
        fromfile='production/simulation.launch.py',tofile='fixture/simulation.launch.py')),encoding='utf-8')
    manifest=dict(scope='test fixture; camera pose and signal only, no production change',
        lamp_pair_center_height_m=.4,camera_xyz_m=[-.075,0.,.30],camera_rpy_rad=[0.,0.,0.],
        provisional=['40cm interpreted as lamp-pair center','20mm vertical pitch = 15mm diameter + 5mm clear gap',
                     'wall-top support and housing','rear camera height and position'],
        removed_links=removed,lamps=lamps,grid_slot=3,world_name=world.get('name'),
        sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (Path(__file__),launch_source,source/'world.sdf',source/'scene.json',source/'centerline.csv',official)})
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(manifest,ensure_ascii=False))


if __name__=='__main__':main()
