"""높은 상대차의 신호 가림: 동일 슬롯3에서 카메라 광학 중심 높이만 정지 비교."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_grid_start import camera_model, put
from audit_start_signal_geometry import rotation


def body(model, name, slot, height, color):
    link=ET.SubElement(model,'link',name=name)
    put(link,'pose',f"{slot['x']} {slot['y']} {height/2} 0 0 {slot['yaw_rad']}")
    visual=ET.SubElement(link,'visual',name=name+'_visual')
    put(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size',f'.20 .15 {height}')
    material=ET.SubElement(visual,'material');put(material,'diffuse',color);put(material,'ambient',color)


def main():
    fixture=BASE/'fixture'
    scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    slots={s['index']:s for s in scene['starting_grid']['slots']}
    for stage in ('red4','green'):
        target=BASE/('tall_'+stage);target.mkdir(exist_ok=False)
        root=ET.parse(fixture/'world.sdf').getroot();world=root.find('world');world.set('name','m5_grid_fixture')
        for p in list(world.findall('plugin')):
            if p.get('name')=='gz::sim::systems::Imu':world.remove(p)
        for e in world.iter():
            if e.tag in ('albedo_map','uri') and e.text:
                asset=(fixture/e.text).resolve();assert asset.is_file();e.text=str(asset)
        for lamp in manifest['lamps']:
            green=lamp['column']==4;on=(stage=='green')==green
            color=('0 1 0 1' if green else '1 0 0 1') if on else '.004 .004 .004 1'
            material=world.find(f"model/link[@name='{lamp['name']}']/visual/material")
            for field in ('ambient','diffuse','emissive'):
                material.find(field).text=color if on or field!='emissive' else '0 0 0 1'
        model=ET.SubElement(world,'model',name='five_static_car_visuals');put(model,'static','true')
        for i in (0,1,2,4):body(model,'peer_'+str(i),slots[i],.40,'.15 .3 .6 1')
        body(model,'ego',slots[3],.12,'.3 .3 .3 1')
        camera=dict(width_px=1280,height_px=720,horizontal_fov_deg=75)
        views={};heights={}
        for height in (.20,.25,.30,.35,.375,.39):
            slot=slots[3];xyz=np.array([slot['x'],slot['y'],0.])+rotation(0.,0.,slot['yaw_rad'])@np.array([-.075,0.,height])
            name=f'h{round(height*1000):03d}';heights[name]=height
            views[name]=camera_model(world,name,xyz,[0.,0.,slot['yaw_rad']],camera)
        xyz=np.array([9.,4.6,1.8]);delta=np.array([9.7,5.9,.25])-xyz
        rpy=[0.,math.atan2(-delta[2],math.hypot(*delta[:2])),math.atan2(delta[1],delta[0])]
        views['overview']=camera_model(world,'overview',xyz,rpy,camera)
        assert len(world.findall('model'))==9 and len(list(world.iter('plugin')))==4
        ET.indent(root);ET.ElementTree(root).write(target/'fixture.sdf',encoding='utf-8',xml_declaration=True)
        paths=[Path(__file__),fixture/'world.sdf',fixture/'scene.json',fixture/'manifest.json']
        report=dict(scope='static five-box geometry, four40cm peers; camera height comparison, not proposed final mount or drive proof',
            stage=stage,render_views=views,camera_heights_m=heights,ego_slot=3,peer_slots=[0,1,2,4],
            own_body_height_m=.12,peer_height_m=.40,control_processes_launched=False,
            mounting_limit='optical center only; camera housing and real body/marker occlusion not modeled',
            source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
        (target/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(str(target))


if __name__=='__main__':main()
