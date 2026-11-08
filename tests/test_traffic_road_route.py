import copy
import pytest
from traffic_road_route import check_onroad_route
import validate_g1_traffic as runner


def geometry():
    config = runner.load_config()
    line, scene = runner.load_centerline()
    case = {'name': 'side_probe', 'route_kind': 'side_road_straight', 'actor_lateral_offset_m': .13,
            'duration_s': 6., 'schedule': [{'from_s': 0., 'to_s': 6., 'speed_mps': .5}]}
    s0 = runner.actor_start_s(config, scene, 0, {'actor_start_center_gap_m': -.3})
    return config, line, case, s0


def test_official_straight_actor_is_wholly_onroad_not_grass():
    config, line, case, start = geometry()
    result = check_onroad_route(config, line, case, start,
        runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf'),
        runner.check_straight, runner.actor_pose_at)
    assert result['actor_onroad_samples'] >= 300
    assert result['minimum_sampled_road_edge_distance_m'] > .01


def test_grass_offset_cannot_be_labeled_onroad():
    config, line, case, start = geometry()
    case['actor_lateral_offset_m'] = .32
    with pytest.raises(ValueError, match='leaves road'):
        check_onroad_route(config, line, case, start,
            runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), [], runner.check_straight, runner.actor_pose_at)


@pytest.mark.parametrize('side', [-1., 1.])
def test_explicit_two_wheel_route_preserves_old_body_metric(side):
    config, line, case, start = geometry()
    config['actor'].update(wheelbase_m=.135, track_m=.135)
    case.update(route_kind='side_two_wheel_straight', actor_lateral_offset_m=side*.20)
    road = runner.road_polygon(runner.DEMO_WORLD/'world.sdf')
    result = runner.check_route(config, line, case, start, road, [])
    assert result['kind'] == 'side_two_wheel_straight'
    assert result['actor_max_off_wheel_centers'] == 2
    assert result['actor_max_outside_body_area_m2'] > 0
    assert result['actor_onroad_samples'] >= 300
    # 새 경로 이름에서만 허용한다. 기존 '도로 안'의 의미는 그대로다.
    case['route_kind'] = 'side_road_straight'
    with pytest.raises(ValueError, match='leaves road'):
        runner.check_route(config, line, case, start, road, [])


@pytest.mark.parametrize('side', [-1., 1.])
def test_two_wheel_route_does_not_allow_four_wheels_out(side):
    config, line, case, start = geometry()
    config['actor'].update(wheelbase_m=.135, track_m=.135)
    case.update(route_kind='side_two_wheel_straight', actor_lateral_offset_m=side*.35)
    with pytest.raises(ValueError, match='leaves road'):
        runner.check_route(config, line, case, start,
            runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), [])


@pytest.mark.parametrize('value', [None, 0., -1., True, float('nan'), .5])
def test_two_wheel_route_requires_explicit_plausible_wheel_geometry(value):
    config, line, case, start = geometry()
    config['actor'].update(wheelbase_m=.135, track_m=value)
    case.update(route_kind='side_two_wheel_straight', actor_lateral_offset_m=.2)
    with pytest.raises(ValueError, match='wheelbase/track'):
        runner.check_route(config, line, case, start,
            runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), [])


def test_two_wheel_permission_never_removes_wall_collision():
    config, line, case, start = geometry()
    config['actor'].update(wheelbase_m=.135, track_m=.135)
    case.update(route_kind='side_two_wheel_straight', actor_lateral_offset_m=.2)
    pose = runner.actor_pose_at(line, start, [], 0., .2)
    obstacle = runner.rectangle(*pose, .2, .15)
    with pytest.raises(ValueError, match='touches wall'):
        runner.check_route(config, line, case, start,
            runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), [obstacle])


def test_new_route_is_explicit_config_only(tmp_path):
    import json
    config, _, case, _ = geometry()
    case['route_kind'] = 'side_two_wheel_straight'
    config['cases'] = [case]
    path = tmp_path/'case.json'
    path.write_text(json.dumps(config))
    assert runner.load_config(path)['cases'][0]['route_kind'] == 'side_two_wheel_straight'
