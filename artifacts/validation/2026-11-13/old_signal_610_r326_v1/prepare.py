"""사용자가 선택한 미확정 구형 610 mm 시설을 별도 실행 사본으로 만든다."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from shapely.affinity import rotate, translate

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
SOURCE = ROOT/'outputs/traffic_light_610_concept_2026_11_13/signal_610_old_style.stl'
EXPECTED = '695621d5dcd3e0278e17c9f036ae5084975ac388897d7ffd1fe507fa4f95e957'
F = ROOT/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1'
spec = importlib.util.spec_from_file_location('prior_fixture', F/'prepare_fixture.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert sha(SOURCE) == EXPECTED
    originals = {str(p.relative_to(ROOT)): sha(p) for p in
                 (SOURCE, WORLD, F/'STardium_starting_light_new.stl', F/'supports_trial.sdf')}
    _, _, tri, _ = prior.analysis.audit.load(SOURCE)
    lo, hi = tri.reshape(-1, 3).min(0), tri.reshape(-1, 3).max(0)
    module = np.all((tri[:, :, 0] > -110) & (tri[:, :, 0] < 90) &
                    (tri[:, :, 1] > -60) & (tri[:, :, 1] < 16), axis=1)
    assert int(module.sum()) == 7033
    center = (lo+hi)/2
    support = tri[~module]
    transformed = np.stack((support[:, :, 2]-center[2], support[:, :, 0]-center[0],
                            support[:, :, 1]-lo[1]), axis=2)/1000
    mesh = A/'support_frame_m.stl'
    prior.write_stl(mesh, transformed)
    # 지난 신형 비교의 위치를 유지한다. ID20 중심은 미확정 가정이다.
    locations = json.loads((F/'fixture.json').read_text())['locations']
    surface = prior.SurfaceMap.from_world(WORLD)
    levels = (.0025, .05, .075, .1346, .198363, .35)
    sections = {h: prior.section(transformed, h) for h in levels}
    sdf = ET.Element('sdf', version='1.10')
    rows = []
    for place in locations:
        model = ET.SubElement(sdf, 'model', name='trial_old610_'+place['name']+'_support')
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f"{place['x']} {place['y']} 0 0 0 {place['yaw']}"
        link = ET.SubElement(model, 'link', name='frame')
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name='support_'+kind)
            geo = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geo, 'uri').text = str(mesh.resolve())
            ET.SubElement(geo, 'scale').text = '1 1 1'
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = '.25 .25 .25 1'
        footprint = []
        for height, shape in sections.items():
            placed = translate(rotate(shape, place['yaw'], origin=(0, 0), use_radians=True),
                               place['x'], place['y'])
            footprint.append(dict(height_m=height, material_area_m2=placed.area,
                road_overlap_m2=placed.intersection(surface.road).area,
                wall_overlap_m2=placed.intersection(surface.walls).area))
        rows.append({**{k: v for k, v in place.items() if k != 'sections'}, 'sections': footprint})
    ET.indent(sdf)
    ET.ElementTree(sdf).write(A/'supports_trial.sdf', encoding='utf-8', xml_declaration=True)
    root = ET.parse(WORLD).getroot()
    world = root.find('world')
    model = world.find("model[@name='it_arena_track_static']")
    # 기존910mm 가로대와 STL의760mm 가로대를 중복 렌더/충돌시키지 않는다.
    beam = model.find("link[@name='tl_beam']")
    assert beam is not None
    model.remove(beam)
    for element in world.iter():
        if element.tag in ('albedo_map', 'uri') and element.text and '://' not in element.text:
            path = (WORLD.parent/element.text).resolve()
            assert path.is_file(), path
            element.text = str(path)
    for model in sdf.findall('model'):
        world.append(copy.deepcopy(model))
    ET.indent(root)
    ET.ElementTree(root).write(A/'world_trial.sdf', encoding='utf-8', xml_declaration=True)
    report = dict(scope='user-selected provisional old610 fixture; not official adoption or driving proof',
        source_sha256=EXPECTED, support_triangles=len(support), removed_module_triangles=int(module.sum()),
        mesh_sha256=sha(mesh), original_sha256=originals,
        raw_to_m=dict(forward='Z-depth centre', left='X-width centre', up='Y-ground minimum',
                      raw_centre_mm=center.tolist(), raw_ground_y_mm=float(lo[1])),
        locations=rows, lighting='existing dynamic 2x5 optical approximation retained, head shift 0; exact lamp optics unverified',
        removed_world_links=['tl_beam'], retained_markers=['0', '45', '20', '30'],
        original_sources_unchanged=all(sha(ROOT/p) == v for p, v in originals.items()))
    assert report['original_sources_unchanged']
    (A/'fixture.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
