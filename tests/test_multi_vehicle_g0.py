import json
import math
from pathlib import Path
import sys

import pytest
import ast
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import validate_multi_vehicle_g0 as g0


def test_eight_cases_preregistered_and_sensor_configuration_unchanged():
    suite = g0.load_suite()
    assert len(suite['cases']) == 8 and suite['seed'] == 0
    assert all(g0.verify_sensor_config(suite)['checks'].values())
    assert g0.baseline_comparison()['unchanged']


def test_trajectory_units_and_stop_restart_phases():
    case = g0.select_cases(g0.load_suite(), 'stop_restart')[0]
    assert [g0.target_at(case, t).x_m for t in (0., 1., 2., 4.)] == pytest.approx([.8, 1.3, 1.3, 2.3])


@pytest.mark.parametrize('case_name', ['empty_r0_corridor', 'low_target'])
@pytest.mark.parametrize('unknown,returns,expected', [
    (0, 417, True), (417, 0, False), (1, 416, False), (0, 0, False),
])
def test_review_zero_target_needs_valid_fixture_observations(case_name, unknown, returns, expected):
    # 닫힌 무잡음 회랑에서는 표적이 없어도 배경 벽 반환이 있어야 한다.
    # 후방 장착 마스크는 unknown이 아니므로 정상 마스크 자체를 실패로 보지 않는다.
    case = g0.select_cases(g0.load_suite(), case_name)[0]
    metrics = dict(visible_target_rays=0, masked_target_rays=0,
                   unknown_rays=unknown, return_rays=returns, masked_rays=83)
    result = g0.visibility_verdict(metrics, case['expected'])
    assert result['passed'] is expected
    assert result['checks']['fixture_observation_complete'] is expected


@pytest.mark.parametrize('case_name', ['empty_r0_corridor', 'low_target'])
def test_review_no_return_analytic_case_cannot_pass(case_name, monkeypatch):
    suite = g0.load_suite()
    monkeypatch.setattr(g0, 'ray_box_distance', lambda *_args, **_kwargs: None)
    result = g0.analytic_case(suite, g0.select_cases(suite, case_name)[0], .135)
    assert result['observation_prediction']['unknown_rays'] > 0
    assert result['observation_prediction']['return_rays'] == 0
    assert not result['analytic_contract_passed']


@pytest.mark.parametrize('pitch', [.05, -.05, 1e-12, math.nan, math.inf, True, '0'])
def test_review_unsupported_pitch_rejected_when_loading_suite(tmp_path, pitch):
    suite = g0.load_suite()
    suite['lidar']['pitch_rad'] = pitch
    path = tmp_path/'tilted.json'
    path.write_text(json.dumps(suite), encoding='utf-8')
    with pytest.raises(ValueError, match='horizontal scan'):
        g0.load_suite(path)


@pytest.mark.parametrize('pitch', [.05, -.05])
def test_review_direct_scene_call_cannot_silently_flatten_tilt(pitch):
    suite = g0.load_suite()
    suite['lidar']['pitch_rad'] = pitch
    with pytest.raises(ValueError, match='horizontal scan'):
        g0.make_scene([1.]*500, [1.]*500, suite, 1.1)


@pytest.mark.parametrize('mode', ['analytic', 'gazebo'])
def test_review_tilted_cli_rejected_before_output_or_simulation(tmp_path, monkeypatch, mode):
    suite = g0.load_suite()
    suite['lidar']['pitch_rad'] = .05
    config_path = tmp_path/'tilted.json'
    config_path.write_text(json.dumps(suite), encoding='utf-8')
    monkeypatch.setattr(g0, 'REPO', tmp_path)
    def must_not_launch(*_args, **_kwargs):
        pytest.fail('invalid fixture started a simulation')
    monkeypatch.setattr(g0, 'gazebo_case', must_not_launch)
    output = tmp_path/'artifacts/not-created'
    with pytest.raises(ValueError, match='horizontal scan'):
        g0.main(['--mode', mode, '--config', str(config_path), '--output', str(output)])
    assert not output.exists()


def test_analytic_cases_scope_and_preregistered_geometry():
    suite = g0.load_suite()
    for case in suite['cases']:
        row = g0.analytic_case(suite, case, .135)
        assert row['analytic_contract_passed'], case['name']
        assert row['synthetic_rays'] and row['rendered_observation_passed'] is None
        assert not row['collision_safety_verified'] and not row['vehicle_stop_verified']
        assert row['r0']['r0_behavior_is_not_g0_tool_verdict']
        assert row['observation_prediction']['source_first_s'] < row['observation_prediction']['t_ref_s']
        if case['name'] == 'empty_r0_corridor':
            assert row['r0']['audit_front_observation']['reason'] == 'ok'
            assert row['r0']['effective_diagnostic_speed_mps'] > 0.


def test_bad_case_does_not_create_output(tmp_path):
    with pytest.raises(ValueError, match='unknown case'):
        g0.main(['--case', 'typo', '--output', str(tmp_path/'not-created')])
    assert not (tmp_path/'not-created').exists()


def test_cli_failure_source_is_separate_and_report_is_preserved(tmp_path, monkeypatch):
    monkeypatch.setattr(g0, 'REPO', tmp_path)
    monkeypatch.setattr(g0, 'snapshot', lambda *_: {})
    monkeypatch.setattr(g0, 'baseline_comparison', lambda *_: {'unchanged': True})
    def unavailable(_):
        raise ImportError('test missing yaml')
    monkeypatch.setattr(g0, 'verify_sensor_config', unavailable)
    output = tmp_path/'artifacts/new'
    assert g0.main(['--output', str(output)]) == 1
    report = json.loads((output/'report.json').read_text())
    assert not report['passed'] and report['failures'][0]['source'] == 'environment'
    assert report['drive_output_topics'] == []
    with pytest.raises(FileExistsError):
        g0.main(['--output', str(output)])


def test_gazebo_world_renders_same_box_as_collision_and_uses_nonduplicated_scan(tmp_path):
    suite = g0.load_suite()
    case = g0.select_cases(suite, 'low_target')[0]
    path = g0.make_world(tmp_path, suite, case)
    world = ET.parse(path).getroot().find('world')
    actor = world.find("model[@name='g0_actor']")
    assert actor.findtext('link/visual/geometry/box/size') == actor.findtext('link/collision/geometry/box/size') == '0.2 0.15 0.06'
    assert actor.findtext("plugin[@name='gz::sim::systems::PosePublisher']/update_frequency") == '100'
    assert world.findtext("model[@name='g0_sensor_rig']/static") == 'true'
    sensor = world.find("model[@name='g0_sensor_rig']/link/sensor")
    assert float(sensor.findtext('update_rate')) == 500.
    assert float(sensor.findtext('lidar/scan/horizontal/max_angle')) == pytest.approx(math.pi-math.tau/500)
    assert world.find("model[@name='g0_wall_3']") is not None  # 후방 end cap
    # PosePublisher는 topic 요소를 지원하지 않는다. 실제 토픽은 model scope에서 정해진다.
    assert actor.find("plugin[@name='gz::sim::systems::PosePublisher']/topic") is None
    assert sensor.findtext('topic') == g0.RAW_TOPIC
    assert world.get('name') == g0.WORLD_NAME


def test_bridge_uses_observed_pose_topic_and_has_no_drive_output():
    arguments = g0.bridge_arguments()
    assert g0.ACTOR_POSE_TOPIC == '/model/g0_actor/pose'
    assert f'{g0.ACTOR_POSE_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' in arguments
    assert f'{g0.RAW_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' in arguments
    assert not any('/g0/actor_pose' in value for value in arguments)
    bridges = arguments[:arguments.index('--ros-args')]
    # 센서/자세는 gz→ROS 단방향, ROS→gz는 평가 actor set_pose 서비스 하나뿐이다.
    assert [value for value in bridges if '[' not in value] == [
        f'{g0.SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean']
    assert not any(token in value for value in arguments for token in ('drive', 'cmd_vel', 'steer'))


@pytest.mark.parametrize('raw,pose,service,name,expected', [
    (None, None, False, 'central_stop', ['raw_scan', 'actor_pose']),
    (.006, None, False, 'central_stop', ['actor_pose']),
    (None, .01, False, 'central_stop', ['raw_scan']),
    # central_v3 실제 순서: 첫 자세 0.010 s가 시작 원시 0.006 s보다 늦으면 시작하지 않는다.
    (.006, .01, False, 'central_stop', ['actor_pose_before_raw_start']),
    (.01, .01, False, 'central_stop', []),
    (.006, None, False, 'empty_r0_corridor', []),
    (.02, .01, False, 'moving_lead', ['actor_set_pose_service']),
    (.02, .01, True, 'stop_restart', []),
])
def test_startup_reports_each_missing_stream(raw, pose, service, name, expected):
    case = g0.select_cases(g0.load_suite(), name)[0]
    assert g0.startup_missing(raw, pose, service, case) == expected


def test_timeout_is_not_classified_as_environment():
    assert g0.failure_source(TimeoutError('startup')) == 'timeout_cause_undetermined'
    assert g0.failure_source(OSError('missing executable')) == 'environment'
    assert g0.failure_source(ImportError('rclpy')) == 'environment'
    assert g0.failure_source(RuntimeError('actor')) == 'harness'


def test_label_tolerance_is_explicit_evaluation_assumption(tmp_path):
    suite = g0.load_suite()
    assert suite['ray_label_match_tolerance_m'] == .02
    assert 'not a sensor' in suite['ray_label_match_status']
    assert 'not measured' in suite['failure_limits_status']
    bad = dict(suite, ray_label_match_tolerance_m=math.inf)
    path = tmp_path/'bad.json'
    path.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match='label tolerance'):
        g0.load_suite(path)


def test_source_timestamp_sidecar_matches_existing_assembler_exact_boundary():
    # ROS 비의존 단위 시험은 생산 class AST만 그대로 실행한다. 생산 코드는 수정하지 않는다.
    source = ROOT/'src/arena_vehicle_interface/arena_vehicle_interface/rotating_lidar.py'
    tree = ast.parse(source.read_text())
    class_node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'RevolutionAssembler')
    scope = {'math': math}
    exec(compile(ast.Module(body=[class_node], type_ignores=[]), str(source), 'exec'), scope)
    assembler = scope['RevolutionAssembler'](rate=10., samples=500, max_gap=.0021)
    raw_times, frames = [], []
    for i in range(102):
        timestamp = 1.+i*.002
        raw_times.append(timestamp)
        frames.extend(assembler.feed(timestamp, [timestamp]*500))
    assert frames
    for first, ranges, _ in frames:
        assert g0.source_ray_times(first, assembler.increment, 500, raw_times) == pytest.approx(ranges, abs=1e-12)


def test_cleanup_only_calls_owned_process(monkeypatch):
    import lidar_shutdown
    touched = []
    monkeypatch.setattr(lidar_shutdown, 'stop_launch', lambda process: touched.append(process) or {})
    monkeypatch.setattr(lidar_shutdown, 'group_processes', lambda _pid: {})
    class Owned:
        returncode = 0
        pid = 12345
    owned = Owned()
    assert g0.process_cleanup(owned)['clean']
    assert touched == [owned]
    assert not g0.process_cleanup(None)['clean']


def test_renderer_request_does_not_count_as_actual_evidence(tmp_path):
    (tmp_path/'simulation.log').write_text('render requested NVIDIA\n', encoding='utf-8')
    assert g0.renderer_evidence(tmp_path, g0.renderer_log_state(tmp_path)) == []


def test_dynamic_phase_no_data_fails():
    suite = g0.load_suite()
    case = g0.select_cases(suite, 'stop_restart')[0]
    result = g0.summarize_phases([], [], case, 1.)
    assert result['required'] and not result['passed']


def test_actor_request_contains_evaluation_box_pose_without_drive():
    from types import SimpleNamespace as N
    request = N(entity=N(name='', type=0, MODEL=2), pose=N(position=N(x=0., y=0., z=0.), orientation=N(z=0., w=0.)))
    target = g0.target_at(g0.select_cases(g0.load_suite(), 'moving_lead')[0], 1.)
    returned = g0.fill_actor_request(request, target)
    assert returned is request and request.entity.name == 'g0_actor' and request.entity.type == 2
    assert request.pose.position.x == pytest.approx(1.3)
    assert request.pose.position.z == pytest.approx(.06)
