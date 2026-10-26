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
