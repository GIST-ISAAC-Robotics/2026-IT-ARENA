"""G1-c 등록 범위 및 추가 설정 보존 검사. 실제 주행은 별도 증거로 남긴다."""
from copy import deepcopy
import math
from pathlib import Path

import pytest
from shapely.geometry import box

import validate_g1_traffic as harness

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'config/tests/g1_traffic_extensions_v1.json'


def test_extra_config_is_in_source_archive():
    paths = harness.source_paths(CONFIG)
    assert 'config/tests/g1_traffic_extensions_v1.json' in paths
    assert len(paths) == len(set(paths))
    with pytest.raises(ValueError):
        harness.source_paths(ROOT.parent/'outside.json')


def test_initial_gap_override_and_original_baseline():
    config = harness.load_config(CONFIG)
    line, scene = harness.load_centerline()
    case = harness.case_named(config, 'initial_occlusion')
    old_start = harness.actor_start_s(config, scene, 0)
    start = harness.actor_start_s(config, scene, 0, case)
    assert old_start-start == pytest.approx(1.05)
    assert harness.check_route(config, line, case, start)['kind'] == 'straight'


def test_registered_curve_route_is_inside_road_and_outside_walls():
    config = harness.load_config(CONFIG)
    line, scene = harness.load_centerline()
    case = harness.case_named(config, 'first_curve')
    start = harness.actor_start_s(config, scene, 0, case)
    road = harness.road_polygon(harness.DEMO_WORLD/'world.sdf')
    walls = harness.obstacle_polygons(harness.DEMO_WORLD/'world.sdf')
    result = harness.check_route(config, line, case, start, road, walls)
    assert result['actor_road_wall_samples'] >= 1200
    assert result['actor_end_s_m'] > result['required_exit_s_m']
    with pytest.raises(ValueError, match='registered straight'):
        harness.check_straight(config, line, case, start)
    with pytest.raises(ValueError):
        harness.check_route(config, line, case, start, None)
    blocked = [box(-100, -100, 100, 100)]
    with pytest.raises(ValueError, match='into wall'):
        harness.check_route(config, line, case, start, road, blocked)
    bad = deepcopy(config)
    bad['acceptance']['first_curve']['maximum_actor_s_m'] = 10.
    with pytest.raises(ValueError, match='extent'):
        harness.check_route(bad, line, case, start, road, walls)


def test_curve_observation_geometry_has_expected_turn():
    config = harness.load_config(CONFIG)
    line, scene = harness.load_centerline()
    start = harness.actor_start_s(config, scene, 0)
    first, last = line.pose(start), line.pose(9.8)
    turn = abs((last[2]-first[2]+math.pi) % math.tau-math.pi)
    assert math.degrees(turn) > 170.
