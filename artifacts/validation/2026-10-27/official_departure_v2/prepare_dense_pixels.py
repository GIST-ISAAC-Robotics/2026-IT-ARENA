"""5대 실패의 출발 정지 자세/재질을 보존한 영상 진단. 주행/출발 성공 시험 아님."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from audit_grid_start import camera_model, put
from audit_start_signal_geometry import rotation


def main():
    fixture = BASE/'fixture'
    scene = json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    manifest = json.loads((fixture/'manifest.json').read_text(encoding='utf-8'))
    actor_source = BASE/'dense5_run1/evaluation_actor.sdf'
    actors = ET.parse(actor_source).getroot()
    for stage in ('red4', 'green'):
        target = BASE/('dense_pixels_'+stage)
        target.mkdir(exist_ok=False)
        root = ET.parse(fixture/'world.sdf').getroot()
        world = root.find('world')
        world.set('name', 'm5_grid_fixture')
        for p in list(world.findall('plugin')):
            if p.get('name') == 'gz::sim::systems::Imu':
                world.remove(p)
        for e in world.iter():
            if e.tag in ('albedo_map', 'uri') and e.text:
                asset = (fixture/e.text).resolve()
                assert asset.is_file()
                e.text = str(asset)
        for lamp in manifest['lamps']:
            green = lamp['column'] == 4
            on = (stage == 'green') == green
            color = ('0 1 0 1' if green else '1 0 0 1') if on else '.004 .004 .004 1'
            material = world.find(f"model/link[@name='{lamp['name']}']/visual/material")
            for field in ('ambient', 'diffuse', 'emissive'):
                material.find(field).text = color if on or field != 'emissive' else '0 0 0 1'
        model = ET.SubElement(world, 'model', name='static_recorded_actor_visuals')
        put(model, 'static', 'true')
        for actor in actors.findall('model'):
            link = ET.SubElement(model, 'link', name=actor.get('name'))
            put(link, 'pose', actor.findtext('pose'))
            link.append(copy.deepcopy(actor.find('link/visual')))
        # 광학 중심은 진행 시험과 같다. 실제 런타임 ego 본체/자기 가림은 이 정지 진단에 포함하지 않는다.
        camera = dict(width_px=1280, height_px=720, horizontal_fov_deg=75)
        views = {}
        for slot in scene['starting_grid']['slots']:
            xyz = np.array([slot['x'], slot['y'], 0.])+rotation(0., 0., slot['yaw_rad'])@np.array([-.075, 0., .30])
            name = 'slot'+str(slot['index'])
            views[name] = camera_model(world, name, xyz, [0., 0., slot['yaw_rad']], camera)
        xyz = np.array([9., 4.6, 1.8])
        delta = np.array([9.7, 5.9, .25])-xyz
        rpy = [0., math.atan2(-delta[2], math.hypot(*delta[:2])), math.atan2(delta[1], delta[0])]
        views['overview'] = camera_model(world, 'overview', xyz, rpy, camera)
        assert len(world.findall('model')) == 9 and len(list(world.iter('plugin'))) == 4
        ET.indent(root)
        ET.ElementTree(root).write(target/'fixture.sdf', encoding='utf-8', xml_declaration=True)
        paths = [Path(__file__), fixture/'world.sdf', fixture/'scene.json', fixture/'manifest.json', actor_source]
        report = dict(stage=stage, render_views=views, relevant_view='slot3',
            scope='recorded four peer initial poses and red materials, static signal pixel diagnosis; no dynamics or start validation',
            limitations='other slot views may be inside a peer; slot3 is the failed ego view; ego self-occlusion excluded',
            source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
        (target/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(target)


if __name__ == '__main__':
    main()
