"""10/29 시험본의 6그리드 정지 빨강/초록 촬영 입력. 주행 프로세스 없음.

official: 공식 ID45 보존. no_id45: 신호 기하만 분리해 보려는 진단 변형이며 실제 시설 제안이 아니다.
"""
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

PEER_HEIGHT=.25
CAMERA=dict(width_px=1280,height_px=720,horizontal_fov_deg=75)


def main():
    fixture=BASE/'fixture';scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest=json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    assert json.loads((BASE/'preflight.json').read_text(encoding='utf-8'))['integrity_passed']
    for variant in ('official','no_id45'):
        for stage in ('red4','green'):
            target=BASE/f'render_{variant}_{stage}';target.mkdir(exist_ok=False)
            root=ET.parse(fixture/'world.sdf').getroot();world=root.find('world');world.set('name','m5_grid_fixture')
            for p in list(world.findall('plugin')):
                if p.get('name')=='gz::sim::systems::Imu':world.remove(p)
            for e in world.iter():
                if e.tag in ('albedo_map','uri') and e.text:
                    asset=(fixture/e.text).resolve();assert asset.is_file();e.text=str(asset)
            track=world.find("model[@name='it_arena_track_static']")
            if variant=='no_id45':track.remove(track.find("link[@name='aruco_45']"))
            for lamp in manifest['lamps']:
                green=lamp['column']==4;on=(stage=='green')==green
                color=('0 1 0 1' if green else '1 0 0 1') if on else '.004 .004 .004 1'
                material=world.find(f"model/link[@name='{lamp['name']}']/visual/material")
                for field in ('ambient','diffuse','emissive'):
                    material.find(field).text=color if on or field!='emissive' else '0 0 0 1'
            # 6칸 모두 20x15x25 cm 상자. 뒤쪽30 cm 광학 중심은 자기 상자 위에 있다. 실제 차량 외형 아님.
            peers=ET.SubElement(world,'model',name='static_grid_boxes');put(peers,'static','true')
            for slot in scene['starting_grid']['slots']:
                link=ET.SubElement(peers,'link',name=f"box_slot{slot['index']}")
                put(link,'pose',f"{slot['x']} {slot['y']} {PEER_HEIGHT/2} 0 0 {slot['yaw_rad']}")
                visual=ET.SubElement(link,'visual',name='body')
                put(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size',f'.20 .15 {PEER_HEIGHT}')
                mat=ET.SubElement(visual,'material');put(mat,'ambient','.2 .2 .25 1');put(mat,'diffuse','.2 .2 .25 1')
            views={}
            for slot in scene['starting_grid']['slots']:
                xyz=np.array([slot['x'],slot['y'],0.])+rotation(0.,0.,slot['yaw_rad'])@np.asarray(manifest['camera_xyz_m'])
                views['slot'+str(slot['index'])]=camera_model(world,'slot'+str(slot['index']),xyz,[0.,0.,slot['yaw_rad']],CAMERA)
            xyz=np.array([9.2,5.0,.9]);delta=np.asarray(manifest['array_center_xyz_m'])-xyz
            rpy=[0.,math.atan2(-delta[2],math.hypot(*delta[:2])),math.atan2(delta[1],delta[0])]
            views['overview']=camera_model(world,'overview',xyz,rpy,CAMERA)
            assert len(world.findall('model'))==9 and len(list(world.iter('plugin')))==4
            ET.indent(root);ET.ElementTree(root).write(target/'fixture.sdf',encoding='utf-8',xml_declaration=True)
            paths=[Path(__file__),fixture/'world.sdf',fixture/'scene.json',fixture/'manifest.json',BASE/'preflight.json']
            report=dict(stage=stage,variant=variant,render_views=views,peer_box_height_m=PEER_HEIGHT,
                scope=f'10/29 tilted 2x5 signal team fixture, {variant}, six 25 cm boxes, rear 30 cm trial camera; static pixels only',
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
            with (target/'report.json').open('w',encoding='utf-8',newline='\n') as stream:
                stream.write(json.dumps(report,ensure_ascii=False,indent=2))
            print(target.relative_to(ROOT).as_posix())


if __name__=='__main__':main()
