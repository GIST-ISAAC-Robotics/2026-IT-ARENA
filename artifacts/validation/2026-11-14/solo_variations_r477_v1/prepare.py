"""평가 전용 배치/벽 변화. 제어기 입력이나 공식 원본을 바꾸지 않는다."""
from pathlib import Path
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
WORLD = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
SCENE = WORLD.with_name('scene.json')
CASES = {
    'start_left20_yaw_plus3': {'lateral_m': .02, 'yaw_delta_deg': 3., 'wall_extra_width_m': 0.},
    'wall_surface_inward10': {'lateral_m': 0., 'yaw_delta_deg': 0., 'wall_extra_width_m': .02},
}


def spawn_variant(x, y, yaw, name):
    case = CASES[name]
    lateral = case['lateral_m']
    return (x-math.sin(yaw)*lateral, y+math.cos(yaw)*lateral,
            yaw+math.radians(case['yaw_delta_deg']))


def world_variant(tree, name):
    """visual 병합 전 트리에 적용한다. 예상한 벽 collision/visual 두 size만 바꾼다."""
    result = copy.deepcopy(tree)
    delta = CASES[name]['wall_extra_width_m']
    changed = []
    if delta:
        for tag, suffix in (('collision', 'col'), ('visual', 'vis')):
            key = f'walls_main_left_1_{suffix}'
            matches = [e for e in result.findall('.//'+tag) if e.get('name') == key]
            if len(matches) != 1:
                raise ValueError(f'exactly one unbatched wall required: {key}')
            size = matches[0].find('geometry/box/size')
            before = [float(v) for v in size.text.split()]
            if len(before) != 3 or any(abs(a-b)>1e-9 for a,b in zip(before,(1.88,.05,.4))):
                raise ValueError(f'wall geometry changed before variation: {before}')
            after = [before[0], before[1]+delta, before[2]]
            size.text = ' '.join(f'{v:.9f}' for v in after)
            changed.append(dict(tag=tag, name=key, before=before, after=after))
    return result, changed


def main():
    output = A/'prepared'
    output.mkdir(exist_ok=False)
    original_bytes = WORLD.read_bytes()
    scene = json.loads(SCENE.read_bytes())
    slot = next(s for s in scene['starting_grid']['slots'] if s['index'] == 3)
    original = ET.fromstring(original_bytes)
    base = (slot['x'], slot['y'], slot['yaw_rad'])
    records = []
    for name, values in CASES.items():
        pose = spawn_variant(*base, name)
        tree, changed = world_variant(original, name)
        # 변경한 size 두 개를 복원했을 때 원 트리와 일치해야 한다.
        restored = copy.deepcopy(tree)
        for item in changed:
            element = next(e for e in restored.findall('.//'+item['tag']) if e.get('name')==item['name'])
            source = next(e for e in original.findall('.//'+item['tag']) if e.get('name')==item['name'])
            element.find('geometry/box/size').text = source.findtext('geometry/box/size')
        assert ET.tostring(restored) == ET.tostring(original)
        assert abs(math.hypot(pose[0]-base[0],pose[1]-base[1])-abs(values['lateral_m'])) < 1e-12
        if changed:
            ET.ElementTree(tree).write(output/f'{name}.sdf', encoding='utf-8', xml_declaration=True)
        records.append(dict(name=name, values=values, spawn=dict(zip(('x','y','yaw_rad'),pose)),
                            world_edits=changed, simulator_executed=False))
    assert WORLD.read_bytes() == original_bytes
    manifest = dict(source_world=str(WORLD.relative_to(ROOT)),
        source_world_sha256=hashlib.sha256(original_bytes).hexdigest(),
        source_scene_sha256=hashlib.sha256(SCENE.read_bytes()).hexdigest(),
        base_spawn=slot, cases=records, changed_source_files=[],
        scope='evaluation-only variations; not connected to launch and not driven',
        requires_successful_nominal_solo=True, independent_control_parameters_unchanged=True)
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(cases=len(records),source_unchanged=True,simulator_executed=False)))


if __name__ == '__main__':
    main()
