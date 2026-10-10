"""G1 잔디 쪽 옆차 시험의 등록 범위·배치·판정 회귀."""
import math
from pathlib import Path

import pytest
from shapely.geometry import box

import validate_g1_traffic as harness
from test_g1_traffic_eval import synthetic_run

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'config/tests/g1_side_occlusion_v1.json'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_side_route_is_wholly_offroad_and_inside_walls(side):
    config = harness.load_config(CONFIG)
    line, scene = harness.load_centerline()
    case = harness.case_named(config, 'side_'+side)
    start = harness.actor_start_s(config, scene, 0, case)
    road = harness.road_polygon(harness.DEMO_WORLD/'world.sdf')
    walls = harness.obstacle_polygons(harness.DEMO_WORLD/'world.sdf')
    result = harness.check_route(config, line, case, start, road, walls)
    assert result['actor_offroad_samples'] > 120
    center = line.pose(start)
    actual = harness.actor_pose_at(line, start, [], 0., harness.actor_lateral_offset(config, case))
    assert math.hypot(actual[0]-center[0], actual[1]-center[1]) == pytest.approx(.32)
    wrong = dict(case, actor_lateral_offset_m=.1)
    with pytest.raises(ValueError, match='not wholly off-road'):
        harness.check_route(config, line, wrong, start, road, walls)
    with pytest.raises(ValueError):
        harness.check_route(config, line, case, start, None, walls)


def side_fixture(actor_x=.32, ego_x=0.):
    config = harness.load_config(CONFIG)
    case = dict(harness.case_named(config, 'side_left'), duration_s=10., schedule=[])
    config['acceptance'][case['name']] = {**config['acceptance'][case['name']], 'minimum_progress_m': 1.5,
        'maximum_stationary_s': 8.}
    actor = [(9.+i*.01, actor_x, 1.5, math.pi/2) for i in range(1400)]
    return synthetic_run(3.2, config=config, case=case, actor=actor, s0=1.5,
        schedule_pose=lambda old: (lambda t: (actor_x, 1.5)), road=box(-.225, -1, .225, 30),
        ego=lambda rows: [(t, ego_x, y, yaw) for t, _, y, yaw in rows])


def test_side_case_keeps_gap_and_road_gates_but_not_behind_rule():
    result = side_fixture()
    assert result['gates']['actor_stays_offroad'] and result['gates']['clears_side_actor']
    assert result['gates']['sampled_gap'] and result['gates']['keeps_original_line']
    assert not result['progress']['ego_stays_behind_actor']
    assert 'ego_stays_behind' not in result['gates']
    assert result['applicability']['ego_stays_behind'] == 'not_applicable_offroad_actor'


def test_side_case_cannot_waive_road_or_gap_or_lateral_limits():
    assert not side_fixture(actor_x=.1)['gates']['actor_stays_offroad']
    assert not side_fixture(actor_x=.25)['gates']['sampled_gap']
    lateral = side_fixture(ego_x=-.1)['gates']
    assert lateral['inside_road'] and not lateral['keeps_original_line']
    assert not side_fixture(ego_x=-.2)['gates']['inside_road']


def test_lateral_actor_cannot_leak_into_original_straight_cases():
    config = harness.load_config(CONFIG)
    assert 'config/tests/g1_side_occlusion_v1.json' in harness.source_paths(CONFIG)
    original = harness.load_config()
    line, _ = harness.load_centerline()
    assert harness.actor_pose_at(line, 3., [], 0.) == line.pose(3.)
    assert harness.actor_lateral_offset(original, original['cases'][0]) == 0.
