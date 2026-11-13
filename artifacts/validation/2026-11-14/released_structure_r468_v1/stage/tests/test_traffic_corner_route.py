"""각본 코너의 연속성·물리속도와 도로/벽 대조. 제어기 시험과 구별한다."""
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle
from traffic_corner_route import corner_geometry, corner_pose, pose_on_corner
from validate_g1_traffic import load_centerline, load_config, actor_start_s, check_route, primary_pose_at, actor_pose_at

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'artifacts/validation/2026-11-08/side_corner_preparation_v1/new_track_course.json'
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'


@pytest.fixture
def setup():
    cfg = load_config(CONFIG)
    line, scene = load_centerline(WORLD)
    case = cfg['cases'][0]
    s0 = actor_start_s(cfg, scene, 0, case)
    return cfg, line, case, s0


def test_registered_route_and_dispatch(setup):
    cfg, line, case, s0 = setup
    result = check_route(cfg, line, case, s0, road_polygon(WORLD/'world.sdf'), obstacle_polygons(WORLD/'world.sdf'))
    assert result['actor_max_off_wheel_centers'] <= 2
    assert result['actor_turn_radius_m'] == pytest.approx(.845)
    assert result['maximum_scheduled_yaw_rate_rad_s'] == pytest.approx(.35/.845)
    for t in (0., 26., 29., 32., 35.):
        assert primary_pose_at(cfg, line, s0, case, t) == corner_pose(cfg, line, s0, case, t)


def test_actual_grid_start_gap_and_rejected_narrower_candidate(setup):
    cfg, line, case, s0 = setup
    scene = json.loads((WORLD/'scene.json').read_bytes())
    slot = scene['starting_grid']['slots'][cfg['launch']['grid_slot']]
    ego = rectangle(slot['x'], slot['y'], slot['yaw_rad'], .20, .15)
    selected = rectangle(*corner_pose(cfg, line, s0, case, 0.), .20, .15)
    assert selected.distance(ego) >= .08
    narrower = {**case, 'actor_lateral_offset_m': -.28}
    rejected = rectangle(*corner_pose(cfg, line, s0, narrower, 0.), .20, .15)
    assert rejected.distance(ego) < .08


def test_tangent_joins_do_not_jump_and_speed_is_physical(setup):
    cfg, line, case, s0 = setup
    g = corner_geometry(cfg, line, case, s0)
    for knot in (g['straight'], g['straight']+g['arc']):
        a, b = pose_on_corner(g, knot-1e-7), pose_on_corner(g, knot+1e-7)
        assert math.dist(a[:2], b[:2]) <= 2.000001e-7
        assert abs(a[2]-b[2]) <= 3e-7
    times = np.linspace(0., 32., 3201)
    poses = np.array([corner_pose(cfg, line, s0, case, t) for t in times])
    speeds = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)/.01
    rates = np.abs(np.diff(np.unwrap(poses[:, 2])))/.01
    assert np.min(speeds) > .34999
    assert np.max(speeds) < .350001
    assert np.max(rates) <= .35/.845+1e-9
    old = np.array([actor_pose_at(line, s0, case['schedule'], t, -.34) for t in times])
    assert np.max(np.linalg.norm(np.diff(old[:, :2], axis=0), axis=1)/.01) > 5.


@pytest.mark.parametrize('key,value', [('turn_rad', float('nan')), ('turn_rad', -math.pi/2),
                                      ('center_radius_m', .1), ('exit_extent_m', 5.)])
def test_unregistered_or_invalid_geometry_rejected(setup, key, value):
    cfg, line, case, s0 = setup
    case = copy.deepcopy(case)
    case['corner_route'][key] = value
    with pytest.raises(ValueError):
        corner_geometry(cfg, line, case, s0)


def test_too_wide_or_incomplete_route_rejected(setup):
    cfg, line, case, s0 = setup
    road, obstacles = road_polygon(WORLD/'world.sdf'), obstacle_polygons(WORLD/'world.sdf')
    too_wide = copy.deepcopy(case)
    too_wide['actor_lateral_offset_m'] = -.45
    with pytest.raises(ValueError):
        check_route(cfg, line, too_wide, s0, road, obstacles)
    incomplete = copy.deepcopy(case)
    incomplete['duration_s'] = 20.
    with pytest.raises(ValueError):
        check_route(cfg, line, incomplete, s0, road, obstacles)


def test_existing_straight_pose_unchanged(setup):
    cfg, line, case, s0 = setup
    case = {**case, 'route_kind': 'side_two_wheel_straight'}
    for t in (0., 1., 10.):
        assert primary_pose_at(cfg, line, s0, case, t) == actor_pose_at(line, s0, case['schedule'], t, -.34)
