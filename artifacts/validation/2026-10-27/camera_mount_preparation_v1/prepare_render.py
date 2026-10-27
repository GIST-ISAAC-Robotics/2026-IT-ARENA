"""공식 배치의 먼 그리드·높이 두 후보·앞차 높이 두 조건. 정지 촬영 준비만 수행."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import yaml

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_grid_start import registry, build_fixture, camera_model
from audit_start_signal_geometry import rotation


def main():
    source = ROOT/'assets/track/official/v2026.10.24/it_arena_track_v2026.10.24.zip'
    with zipfile.ZipFile(source) as archive:
        raw = archive.read(next(n for n in archive.namelist() if n.endswith('world.sdf')))
        scene = json.loads(archive.read(next(n for n in archive.namelist() if n.endswith('scene.json'))))
    vehicle_path = ROOT/'src/arena_description/config/vehicle.yaml'
    camera_path = ROOT/'src/arena_description/config/b0183_c1.yaml'
    vehicle = yaml.safe_load(vehicle_path.read_text())['vehicle']
    baseline = yaml.safe_load(camera_path.read_text())['camera']
    rows = registry(scene,raw,vehicle)
    ego = next(r for r in rows if r['index']==3)
    chosen = [r for r in rows if r['index'] in (2,5,1,4,3)]
    hashes = {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (Path(__file__),source,vehicle_path,camera_path,
                        ROOT/'scripts/audit_grid_start.py',ROOT/'scripts/capture_grid_start.py')}
    for height in (.25,.40):
        for stage in ('red4','green'):
            directory = OUT/f'render_h{int(height*100):02}_{stage}'
            directory.mkdir(exist_ok=False)
            root,views = build_fixture(raw,chosen,vehicle,baseline,dict(height_m=height,bottom_m=0.))
            world = root.find('world')
            # 자차 형상은 .12 m 상자. 상대차 높이는 가정 범위이며 공식 형상이 아니다.
            world.find("model[@name='eval_grid_3']/pose").text = f"{ego['x']} {ego['y']} .06 0 0 {ego['yaw_rad']}"
            world.find("model[@name='eval_grid_3']/link/visual/geometry/box/size").text = '.20 .15 .12'
            for model in list(world.findall('model')):
                if model.get('name').startswith('m5_camera_') and model.get('name')!='m5_camera_overview':
                    world.remove(model)
            views = {'overview':views['overview']}
            for z,up in ((.20,25.328),(.30,21.910)):
                cam = copy.deepcopy(baseline)
                cam['xyz_m'] = [-.075,0.,z]
                cam['rpy_rad'] = [0.,-math.radians(up),0.]
                xyz = np.array([ego['x'],ego['y'],0.])+rotation(0,0,ego['yaw_rad'])@cam['xyz_m']
                name = f'rear_{int(z*100)}cm'
                views[name] = camera_model(world,name,xyz,[0.,cam['rpy_rad'][1],ego['yaw_rad']],cam)
            for link in world.findall("model[@name='it_arena_track_static']/link"):
                name = link.get('name','')
                if not name.startswith('lamp_'):
                    continue
                material = link.find('visual/material')
                on = (stage=='green') == (name=='lamp_green')
                color = ('0 1 0 1' if name=='lamp_green' else '1 0 0 1') if on else '.004 .004 .004 1'
                for field in ('ambient','diffuse','emissive'):
                    node = material.find(field)
                    if node is None:
                        node = ET.SubElement(material,field)
                    node.text = color if on or field!='emissive' else '0 0 0 1'
            assert len(world.findall('model')) == 9
            assert all(m.findtext('static')=='true' for m in world.findall('model'))
            ET.indent(root)
            ET.ElementTree(root).write(directory/'fixture.sdf',encoding='utf-8',xml_declaration=True)
            report = dict(scope='official_pose_static_lamp_material_stages_and_box_occlusion_no_driving',
                ego_grid=3,actor_height_m=height,ego_height_m=.12,stage=stage,
                field_slots=[r['index'] for r in chosen],render_views=views,source_sha256=hashes,
                caveat='separate static material stages, no live controller or light switching; mount not adopted')
            (directory/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            print(directory.relative_to(ROOT).as_posix())


if __name__ == '__main__':
    main()
