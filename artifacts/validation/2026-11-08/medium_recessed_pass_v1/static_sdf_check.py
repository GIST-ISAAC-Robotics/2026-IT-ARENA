"""정적 XML 대조: 현재 actor_sdf로 r15 설정(box)과 새 설정(recessed_sides)을 같은 시작 자세에서 생성한다.

- box 생성본이 r15 실제 run1/evaluation_actor.sdf와 바이트 동일한지(현재 코드가 r15 상대를 그대로 만드는지)
- 두 생성본에서 visual 요소만 빼면 동일한지(충돌·접촉 센서·static·pose·플러그인 불변)
- 오목 visual 합집합이 외접 상자 안이고 중앙 폭 2/3·앞뒤 끝 원폭인지
사용: check(run_dir) 형태로 실행 후 runtime SDF도 같은 방법으로 대조한다.
"""
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import validate_g1_traffic as harness

HERE = Path(__file__).resolve().parent
R15 = HERE.parent/'medium_pass_connected_v1'
POSE = (9.92, 5.12, 1.570796327)


def without_visuals(text):
    root = ET.fromstring(text)
    link = root.find('model/link')
    for visual in link.findall('visual'):
        link.remove(visual)
    return ET.tostring(root, encoding='unicode')


def visuals(text):
    link = ET.fromstring(text).find('model/link')
    out = []
    for v in link.findall('visual'):
        pose = [float(x) for x in (v.findtext('pose') or '0 0 0 0 0 0').split()]
        size = [float(x) for x in v.findtext('geometry/box/size').split()]
        out.append(dict(name=v.get('name'), pose=pose, size=size))
    return out


def shape_checks(vis):
    xs = [(v['pose'][0]-v['size'][0]/2, v['pose'][0]+v['size'][0]/2) for v in vis]
    ys = [(v['pose'][1]-v['size'][1]/2, v['pose'][1]+v['size'][1]/2) for v in vis]
    center = next(v for v in vis if v['name'] == 'visual_center')
    ends = [v for v in vis if v['name'] in ('visual_front', 'visual_rear')]
    return dict(
        bounds_m=[min(a for a, _ in xs), min(a for a, _ in ys), max(b for _, b in xs), max(b for _, b in ys)],
        center_width_m=center['size'][1], center_length_m=center['size'][0],
        end_widths_m=[v['size'][1] for v in ends], heights_m=sorted({v['size'][2] for v in vis}),
        z_and_rotation_zero=all(v['pose'][2:] == [0., 0., 0., 0.] for v in vis))


def main(run=None):
    prior = harness.load_config(R15/'new_track_course.json')
    current = harness.load_config(HERE/'new_track_course.json')
    box, recessed = harness.actor_sdf(prior, POSE), harness.actor_sdf(current, POSE)
    (HERE/'static_box_current_code.sdf').write_text(box, encoding='utf-8', newline='\n')
    (HERE/'static_recessed_current_code.sdf').write_text(recessed, encoding='utf-8', newline='\n')
    r15_runtime = (R15/'straight_moving_lead/run1/evaluation_actor.sdf').read_text(encoding='utf-8')
    vis = visuals(recessed)
    shape = shape_checks(vis)
    result = dict(
        pose=POSE,
        box_equals_r15_runtime_bytes=box == r15_runtime,
        non_visual_xml_identical=without_visuals(box) == without_visuals(recessed),
        box_visuals=visuals(box), recessed_visuals=vis, recessed_shape=shape,
        recessed_inside_box=shape['bounds_m'] == [-.1, -.075, .1, .075] or all(
            abs(a-b) < 1e-9 for a, b in zip(shape['bounds_m'], [-.1, -.075, .1, .075])),
        center_two_thirds=abs(shape['center_width_m']-.1) < 1e-9,
        ends_full_width=all(abs(w-.15) < 1e-9 for w in shape['end_widths_m']))
    if run is not None:
        runtime = (Path(run)/'evaluation_actor.sdf').read_text(encoding='utf-8')
        result['runtime'] = dict(
            path=str(Path(run)/'evaluation_actor.sdf'),
            equals_static_recessed_bytes=runtime == recessed,
            non_visual_equals_r15_runtime=without_visuals(runtime) == without_visuals(r15_runtime),
            visuals_equal_static=visuals(runtime) == vis)
    keys = ['box_equals_r15_runtime_bytes', 'non_visual_xml_identical', 'recessed_inside_box',
            'center_two_thirds', 'ends_full_width']
    result['passed'] = all(result[k] for k in keys) and all(
        v for k, v in result.get('runtime', {}).items() if k != 'path')
    name = 'static_sdf_check.json' if run is None else 'runtime_sdf_check.json'
    (HERE/name).write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('box_visuals', 'recessed_visuals')}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else None))
