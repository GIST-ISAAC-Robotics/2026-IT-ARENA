"""평가용 관측 윤곽: 기존 box는 불변, 오목 시각 외형도 외접 상자 충돌/간격을 유지한다."""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from shapely.geometry import LineString, box
from shapely.ops import unary_union

import validate_g1_traffic as harness

ROOT = Path(__file__).resolve().parents[1]


def test_unspecified_outline_preserves_old_sdf_byte_for_byte():
    old = (ROOT/'artifacts/validation/2026-11-03/irregular_actor_v1/legacy_actor.sdf').read_text()
    cfg = harness.load_config(ROOT/'config/tests/official_traffic_v1.json')
    assert harness.actor_sdf(cfg, (10., 6., math.pi/2)) == old
    cfg['actor']['visual_outline'] = 'box'
    assert harness.actor_sdf(cfg, (10., 6., math.pi/2)) == old


def test_recessed_outline_is_connected_and_inside_unchanged_contact_box():
    cfg = harness.load_config(ROOT/'config/tests/irregular_actor_v1.json')
    root = ET.fromstring(harness.actor_sdf(cfg, (10., 6., math.pi/2)))
    link = root.find('model/link')
    assert len(link.findall('collision')) == 1
    assert [float(v) for v in link.findtext('collision/geometry/box/size').split()] == [.2, .15, .12]
    assert link.findtext('sensor/contact/collision') == 'collision'
    assert link.findtext('sensor/contact/topic') == harness.CONTACT_TOPIC
    shapes = []
    for visual in link.findall('visual'):
        x, y, z, roll, pitch, yaw = map(float, visual.findtext('pose').split())
        length, width, height = map(float, visual.findtext('geometry/box/size').split())
        assert (z, roll, pitch, yaw) == (0., 0., 0., 0.) and height == .12
        shapes.append(box(x-length/2, y-width/2, x+length/2, y+width/2))
    shape = unary_union(shapes)
    assert shape.geom_type == 'Polygon' and shape.is_valid
    assert shape.bounds == pytest.approx((-.1, -.075, .1, .075))
    assert box(-.1, -.075, .1, .075).buffer(1e-12).covers(shape)
    # 중앙 옆면은 25 mm 들어가지만 앞뒤 폭은 그대로이며, 전체 차량을 좁힌 시험은 아니다.
    assert LineString([(0., -.2), (0., .2)]).intersection(shape).length == pytest.approx(.10)
    assert LineString([(.09, -.2), (.09, .2)]).intersection(shape).length == pytest.approx(.15)


def test_registered_case_keeps_prior_schedule_physics_and_judgment():
    prior = harness.load_config(ROOT/'config/tests/official_traffic_v1.json')
    current = harness.load_config(ROOT/'config/tests/irregular_actor_v1.json')
    assert current['cases'] == [prior['cases'][0]]
    for key in ('launch', 'acceptance', 'seeds', 'timeouts'):
        assert current[key] == prior[key]
    actor = deepcopy(current['actor'])
    actor.pop('visual_outline')
    actor['visible_section'] = prior['actor']['visible_section']
    assert actor == prior['actor']


def test_unknown_outline_does_not_silently_fall_back_to_box():
    cfg = harness.load_config(ROOT/'config/tests/irregular_actor_v1.json')
    cfg['actor']['visual_outline'] = 'typo'
    with pytest.raises(ValueError, match='outline'):
        harness.actor_sdf(cfg, (0., 0., 0.))
