"""팀 비교본의 잔디 링크만 1 mm 올린다. 공식 원본·도로·방지턱·물성은 보존한다."""
from copy import deepcopy
import hashlib
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def level_tree(root):
    grass = root.findall(".//model/link[@name='grass']")
    roads = root.findall(".//model/link[@name='track_surface']")
    if len(grass) != 1 or len(roads) != 1:
        raise ValueError('expected one explicit grass and road link')
    grass, road = grass[0], roads[0]
    for link in (grass, road):
        if link.find('pose') is not None:
            raise ValueError('unexpected link transform; do not guess composition')
        for model in root.findall('.//model'):
            if link in list(model) and model.find('pose') is not None:
                raise ValueError('unexpected model transform')

    def top(link):
        collisions = link.findall('collision')
        if not collisions:
            raise ValueError('missing surface collisions')
        values = []
        for item in collisions + link.findall('visual'):
            shape = item.findtext('geometry/box/size')
            pose = item.findtext('pose')
            if shape is None or pose is None:
                raise ValueError('expected explicit horizontal box')
            size, xyzrpy = list(map(float, shape.split())), list(map(float, pose.split()))
            if len(size) != 3 or len(xyzrpy) != 6 or any(abs(v) > 1e-12 for v in xyzrpy[3:5]):
                raise ValueError('nonhorizontal surface box')
            values.append(xyzrpy[2]+size[2]/2)
        if max(values)-min(values) > 1e-12:
            raise ValueError('mixed surface heights')
        return values[0]

    grass_top, road_top = top(grass), top(road)
    if abs(grass_top-.002) > 1e-12 or abs(road_top-.003) > 1e-12:
        raise ValueError('not the registered 2 mm grass / 3 mm road baseline')
    before = deepcopy(root)
    pose = ET.SubElement(grass, 'pose')
    pose.text = '0 0 0.001 0 0 0'
    checked = deepcopy(root)
    checked_grass = checked.findall(".//model/link[@name='grass']")[0]
    checked_grass.remove(checked_grass.find('pose'))
    if ET.tostring(before) != ET.tostring(checked):
        raise AssertionError('unexpected world change')
    return dict(original_grass_top_m=grass_top, road_top_m=road_top,
                variant_grass_top_m=grass_top+.001, grass_shift_m=.001,
                collision_boxes=len(grass.findall('collision')), visual_boxes=len(grass.findall('visual')),
                only_grass_link_pose_changed=True, original_source_preserved=True,
                actual_track_flatness_claimed=False)


def build_fixture(source, target):
    source, target = Path(source), Path(target)
    if target.exists():
        raise FileExistsError(target)
    original = (source/'world.sdf').read_bytes()
    root = ET.fromstring(original)
    result = level_tree(root)
    shutil.copytree(source, target)
    ET.ElementTree(root).write(target/'world.sdf', encoding='utf-8', xml_declaration=True)
    assert (source/'world.sdf').read_bytes() == original
    other = [p for p in source.rglob('*') if p.is_file() and p.name != 'world.sdf']
    assert all(p.read_bytes() == (target/p.relative_to(source)).read_bytes() for p in other)
    result.update(original_world_sha256=hashlib.sha256(original).hexdigest(),
                  variant_world_sha256=hashlib.sha256((target/'world.sdf').read_bytes()).hexdigest(),
                  other_fixture_files_unchanged=len(other))
    return result
