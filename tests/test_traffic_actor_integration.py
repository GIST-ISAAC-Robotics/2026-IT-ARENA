"""공식 월드·기존 생성기를 이용한 복수 actor 사전 검사. ROS/Gazebo 실행과 별도다."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import traffic_actors as ta
import validate_g1_traffic as runner


def test_actual_sdf_composition_and_offroad_route_is_labeled():
    config = runner.load_config()
    case = ta.overlay_case(runner.case_named(config, 'stopped_center'), ROOT/'config/tests/e3_two_actors_harness_v1.json')
    specs = ta.actor_specs(config, case)
    line, scene = runner.load_centerline()
    starts, routes = ta.check_additional_routes(config, case, specs, scene, 0, line,
        runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf'),
        runner.actor_start_s, runner.check_route, runner.actor_pose_at)
    assert routes['sampled_actor_pair_min_gap_m'] > .15
    assert routes['actors']['eval_e3_side']['kind'] == 'side_grass_straight'
    poses = [runner.actor_pose_at(line, s, a['schedule'], 0., a['lateral_offset_m']) for s, a in zip(starts, specs)]
    root = ET.fromstring(ta.compose_sdf(specs, poses, runner.actor_sdf))
    assert len(root.findall('model')) == 2 and len(root.findall('plugin')) == 1
    assert all(m.findtext('static') == 'true' for m in root.findall('model'))
    assert len({m.findtext('link/sensor/contact/topic') for m in root.findall('model')}) == 2


def test_overlapping_actor_schedule_rejected_before_gazebo():
    config = runner.load_config()
    case = ta.overlay_case(runner.case_named(config, 'stopped_center'), ROOT/'config/tests/e3_two_actors_harness_v1.json')
    case['additional_actors'][0].update(start_center_gap_m=1.60, lateral_offset_m=0., route_kind='straight')
    specs = ta.actor_specs(config, case)
    line, scene = runner.load_centerline()
    with pytest.raises(ValueError, match='schedules overlap'):
        ta.check_additional_routes(config, case, specs, scene, 0, line,
            runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), [],
            runner.actor_start_s, runner.check_route, runner.actor_pose_at)


def test_onroad_overlay_retains_gap_and_actor_schedule_has_no_peer_overlap():
    config = runner.load_config()
    case = ta.overlay_case(runner.case_named(config, 'slow_lead'), ROOT/'config/tests/e3_onroad_encounter_v1.json')
    specs = ta.actor_specs(config, case)
    line, scene = runner.load_centerline()
    _, routes = ta.check_additional_routes(config, case, specs, scene, 0, line,
        runner.road_polygon(runner.DEMO_WORLD/'world.sdf'), runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf'),
        runner.actor_start_s, runner.check_route, runner.actor_pose_at)
    assert specs[1]['minimum_body_gap_m'] == .15
    assert routes['actors']['eval_e3_side']['minimum_sampled_road_edge_distance_m'] > .01
    assert routes['sampled_actor_pair_min_gap_m'] > .7
