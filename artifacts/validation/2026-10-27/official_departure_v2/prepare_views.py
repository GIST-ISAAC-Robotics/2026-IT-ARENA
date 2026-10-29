"""새 시험 설치의6그리드 정지 빨강/초록 촬영본. 주행 프로세스는 없다."""
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


def main():
    fixture=BASE/'fixture';scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    assert json.loads((BASE/'preflight.json').read_text(encoding='utf-8'))['passed']
    camera=dict(xyz_m=manifest['camera_xyz_m'],rpy_rad=manifest['camera_rpy_rad'],width_px=1280,height_px=720,horizontal_fov_deg=75)
    for stage in ('red4','green'):
        target=BASE/('render_'+stage);target.mkdir(exist_ok=False)
        root=ET.parse(fixture/'world.sdf').getroot();world=root.find('world');world.set('name','m5_grid_fixture')
        for p in list(world.findall('plugin')):
            if p.get('name')=='gz::sim::systems::Imu':world.remove(p)
        for e in world.iter():
            if e.tag in ('albedo_map','uri') and e.text:
                asset=(fixture/e.text).resolve();assert asset.is_file();e.text=str(asset)
        for lamp in manifest['lamps']:
            is_green=lamp['column']==4;on=(stage=='green')==is_green
            color=('0 1 0 1' if is_green else '1 0 0 1') if on else '.004 .004 .004 1'
            material=world.find(f"model/link[@name='{lamp['name']}']/visual/material")
            for field in ('ambient','diffuse','emissive'):
                material.find(field).text=color if on or field!='emissive' else '0 0 0 1'
        views={}
        for slot in scene['starting_grid']['slots']:
            xyz=np.array([slot['x'],slot['y'],0.])+rotation(0.,0.,slot['yaw_rad'])@camera['xyz_m']
            name='slot'+str(slot['index'])
            views[name]=camera_model(world,name,xyz,[0.,0.,slot['yaw_rad']],camera)
        # 실제 장착 기구 대신 v1과 같은 낮은 자차 상자만 넣는다. 다른 차량은 없다.
        slot=next(s for s in scene['starting_grid']['slots'] if s['index']==3)
        model=ET.SubElement(world,'model',name='ego_body_reference');put(model,'static','true')
        put(model,'pose',f"{slot['x']} {slot['y']} .06 0 0 {slot['yaw_rad']}")
        link=ET.SubElement(model,'link',name='body');visual=ET.SubElement(link,'visual',name='body')
        put(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size','.20 .15 .12')
        xyz=np.array([9.,4.6,1.8]);center=np.array([9.7,5.9,.25]);delta=center-xyz
        rpy=[0.,math.atan2(-delta[2],math.hypot(*delta[:2])),math.atan2(delta[1],delta[0])]
        views['overview']=camera_model(world,'overview',xyz,rpy,camera)
        assert len(world.findall('model'))==9 and len(list(world.iter('plugin')))==4
        ET.indent(root);ET.ElementTree(root).write(target/'fixture.sdf',encoding='utf-8',xml_declaration=True)
        paths=[Path(__file__),fixture/'world.sdf',fixture/'scene.json',fixture/'manifest.json',
               ROOT/'scripts/audit_grid_start.py',ROOT/'scripts/capture_grid_start.py',*fixture.rglob('*.png'),*fixture.rglob('*.stl')]
        report=dict(scope='six grid static views of provisional left signal, one low ego box, no peer cars/no driving',
                    stage=stage,render_views=views,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
        (target/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(target.relative_to(ROOT).as_posix())


if __name__=='__main__':main()
