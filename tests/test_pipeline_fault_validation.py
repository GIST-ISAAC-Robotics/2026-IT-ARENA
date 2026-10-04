"""ROS 실행 없이 검증기의 측정/판정 경계와 가상 센서 fixture를 검사한다."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/validate_pipeline_faults_ros.py'
spec = importlib.util.spec_from_file_location('pipeline_fault_validation', SCRIPT)
validation = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = validation
spec.loader.exec_module(validation)


def test_finite_corridor_uses_wall_intersections_and_lidar_offset():
    ray = validation.corridor_range
    assert ray(0, .06) == pytest.approx(5.94)
    assert ray(math.pi, .06) == pytest.approx(6.06)
    assert ray(math.pi / 2) == pytest.approx(.425)
    assert ray(-math.pi / 2) == pytest.approx(.425)
    ranges = [ray(-math.pi + n * math.tau / 500, .06) for n in range(500)]
    assert all(.05 <= value <= 12. and math.isfinite(value) for value in ranges)
    with pytest.raises(ValueError):
        ray(0, origin_y=.425)


def test_toy_stop_measures_model_distance_and_does_not_instantly_brake():
    plant = validation.ToyPlant(speed=.6)
    plant.step(0.)
    assert plant.speed == pytest.approx(.59)
    assert plant.distance > 0
    for _ in range(100):
        plant.step(0.)
    assert plant.speed == 0.
    assert plant.distance == pytest.approx(.09)
    stopped_distance = plant.distance
    plant.step(0.)
    assert plant.distance == stopped_distance
    with pytest.raises(ValueError):
        plant.step(math.nan)


def test_old_zero_status_cannot_satisfy_a_new_stop_case(monkeypatch):
    monkeypatch.setattr(validation.time, 'monotonic', lambda: 10.)
    monitor = validation.Monitor()
    monitor.receive('mcu', {'state': 'FAULT_LATCHED', 'target_speed_mps': 0.}, 1., 1.)
    baseline = monitor.sequence
    zero = lambda value: value['target_speed_mps'] == 0.
    assert not monitor.fresh('mcu', baseline, zero, 10., 1.)
    monitor.receive('mcu', {'state': 'ACTIVE', 'target_speed_mps': .6}, 1.01, 1.01)
    assert not monitor.fresh('mcu', baseline, zero, 10., 1.01)
    monitor.receive('mcu', {'state': 'FAULT_LATCHED', 'target_speed_mps': 0.}, 1.02, 1.02)
    assert monitor.fresh('mcu', baseline, zero, 10., 1.02)


def test_fresh_receipt_does_not_accept_old_source_or_stale_monitor(monkeypatch):
    monkeypatch.setattr(validation.time, 'monotonic', lambda: 10.)
    monitor = validation.Monitor()
    monitor.receive('safe', 0., 2., 1.)
    assert not monitor.fresh('safe', 0, lambda v: v == 0., 10., 2.)
    monitor.receive('safe', 0., 2., 2.)
    assert monitor.fresh('safe', 0, lambda v: v == 0., 10., 2.)
    assert not monitor.fresh('safe', 0, lambda v: v == 0., 10.6, 2.)
    assert not monitor.fresh('safe', 0, lambda v: v == 0., 10., 2.3)


def test_pid_reuse_is_rejected_even_if_the_pid_and_group_are_the_same():
    identity = dict(pid=42, start_ticks=123, group=42, session=42)
    assert validation.same_process(identity, identity.copy())
    reused = {**identity, 'start_ticks': 456}
    assert not validation.same_process(identity, reused)


def test_signal_blocks_reparented_vision_without_touching_process(monkeypatch, tmp_path):
    report = {}
    owned = validation.OwnedProcesses(tmp_path, report)
    identity = dict(pid=42, parent=10, start_ticks=123, group=10, session=10)
    owned.current['controller'] = {'child': SimpleNamespace(pid=10),
                                   'record': {'identity': {**identity, 'pid': 10}}}
    monkeypatch.setattr(validation, 'proc_identity', lambda pid: {**identity, 'parent': 99})
    touched = []
    monkeypatch.setattr(validation.os, 'kill', lambda *args: touched.append(args))
    with pytest.raises(RuntimeError, match='parent changed'):
        owned.send('controller', validation.signal.SIGCONT, vision=identity)
    assert not touched


def test_pixel_fixture_is_read_by_actual_start_detector():
    pytest.importorskip('cv2')
    pytest.importorskip('numpy')
    sys.path.insert(0, str(SCRIPT.parents[1] / 'src/arena_autonomy'))
    from arena_autonomy.core import StartSignal, image_rgb
    detector = StartSignal()
    for color in ('green', 'red', 'red', 'green', 'green', 'green'):
        message = SimpleNamespace(height=480, width=848, step=848 * 3, encoding='rgb8',
                                  data=validation.signal_rgb(color))
        detector.update(image_rgb(message))
        if color == 'red':
            assert not detector.started
    assert detector.started


def zero_monitor(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: clock[0])
    monitor = validation.Monitor()
    for key, value in [('applied', 0.), ('sink', {'speed_mps': 0.}), ('mcu', {'target_speed_mps': 0.})]:
        monitor.receive(key, value, 1., None if key == 'applied' else 1.)
    case = dict(sequence=monitor.sequence)
    monitor.begin_zero_watch(case)
    return monitor, clock, case


def test_no_resume_rejects_transient_positive_even_after_later_zero(monkeypatch):
    monitor, clock, case = zero_monitor(monkeypatch)
    monitor.receive('applied', .1, 1.01)
    monitor.receive('applied', 0., 1.02)
    with pytest.raises(AssertionError, match='nonzero excursion'):
        monitor.check_zero_watch(clock[0], 1.02)
    assert case['nonzero_excursion']['value'] == .1


def test_stop_observation_requires_every_topic_and_rejects_missing_receipts(monkeypatch):
    monitor, clock, _ = zero_monitor(monkeypatch)
    monitor.receive('applied', 0., 1.01)
    monitor.check_zero_watch(clock[0], 1.01)
    with pytest.raises(AssertionError, match='every output topic'):
        monitor.end_zero_watch()
    monitor.receive('applied', 0., 1.26)
    monitor.receive('sink', {'speed_mps': 0.}, 1.26, 1.26)
    with pytest.raises(AssertionError, match='gap|missing'):
        monitor.check_zero_watch(clock[0], 1.26)


def test_zero_receipt_gap_is_rejected_even_when_latest_receipt_is_fresh(monkeypatch):
    monitor, clock, _ = zero_monitor(monkeypatch)
    clock[0] += validation.ZERO_GAP_WALL_S + .01
    monitor.receive('applied', 0., 1.01)
    with pytest.raises(AssertionError, match='receipt gap'):
        monitor.check_zero_watch(clock[0], 1.01)


def fake_validator(monkeypatch):
    wall = [10.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: wall[0])
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.monitor = validation.Monitor()
    validator.plant = validation.ToyPlant()
    validator.t = 1.
    validator.report = dict(cases=[])
    validator.children = SimpleNamespace(children=[])

    def receive_outputs(speed=0.):
        validator.monitor.receive('applied', speed, validator.t)
        validator.monitor.receive('sink', {'speed_mps': speed}, validator.t, validator.t)
        validator.monitor.receive('mcu', {'target_speed_mps': speed}, validator.t, validator.t)

    def tick():
        wall[0] += .01
        validator.t += .01
        receive_outputs()
        validator.monitor.check_zero_watch(wall[0], validator.t)

    def wait(predicate, label, timeout=15.):
        for _ in range(100):
            tick()
            if predicate():
                return
        raise AssertionError('timeout: ' + label)

    def pump(duration):
        for _ in range(round(duration / .01)):
            tick()

    validator.wait, validator.pump = wait, pump
    receive_outputs()
    return validator, receive_outputs


def test_no_resume_initial_state_must_be_fresh_and_stopped(monkeypatch):
    validator, receive_outputs = fake_validator(monkeypatch)
    receive_outputs(.1)
    actions = []
    with pytest.raises(AssertionError, match='initial stopped'):
        validator.stop_case('restore', lambda: actions.append(True), {}, no_resume=True)
    assert not actions


def test_no_resume_action_is_inside_the_zero_observation_boundary(monkeypatch):
    validator, receive_outputs = fake_validator(monkeypatch)

    def restore():
        receive_outputs(.2)
        receive_outputs(0.)

    with pytest.raises(AssertionError, match='nonzero excursion'):
        validator.stop_case('restore', restore, {}, no_resume=True)
    assert not validator.report['cases'][0]['passed']


def test_no_resume_zero_case_requires_all_three_fresh_at_tail(monkeypatch):
    validator, _ = fake_validator(monkeypatch)
    case = validator.stop_case('restore', lambda: None, {}, no_resume=True)
    assert case['passed']
    assert all(case['zero_observation']['counts'][key] > 0 for key in validation.ZERO_TOPICS)
    assert all(key in case['final'] for key in validation.ZERO_TOPICS)
    assert validator.monitor.zero_watch is None


@pytest.mark.parametrize('failure', ['plant_motion', 'sink_nonzero', 'sink_missing'])
def test_final_tail_requires_stopped_plant_and_every_fresh_output(monkeypatch, failure):
    validator, _ = fake_validator(monkeypatch)
    case = validator.mark('tail')

    def wait(predicate, label, timeout=15.):
        validator.monitor.receive('applied', 0., validator.t)
        validator.monitor.receive('mcu', {'target_speed_mps': 0.}, validator.t, validator.t)
        if failure != 'sink_missing':
            validator.monitor.receive('sink', {'speed_mps': .2 if failure == 'sink_nonzero' else 0.},
                                      validator.t, validator.t)
        if failure == 'plant_motion':
            validator.plant.speed = .2
        assert not predicate()
        raise AssertionError('tail not stopped')

    validator.wait = wait
    with pytest.raises(AssertionError, match='tail not stopped'):
        validator.finish(case, expected={'applied': lambda v: v == 0.}, duration=0., require_stop=True)
    assert not case['passed']


@pytest.mark.parametrize(('forced', 'code', 'intentional', 'accepted'), [
    (True, -15, False, False), (True, -9, True, False), (False, -9, False, False),
    (False, -9, True, True), (False, 0, False, True), (False, 1, True, False)])
def test_vision_cleanup_distinguishes_intentional_kill_from_forced_cleanup(forced, code, intentional, accepted):
    line = 'warning Vision process cleanup: ' + repr(dict(forced=forced, exitcode=code))
    assert validation.vision_cleanup_results([line], intentional)[0]['accepted'] == accepted


def test_pre_injection_source_zero_is_excluded_from_first_zero(monkeypatch):
    validator, _ = fake_validator(monkeypatch)
    case = validator.mark('fault')
    validator.monitor.receive('safe', 0., 1.01, .99)
    validator.monitor.receive('safe', 0., 1.02, 1.01)
    validator.monitor.receive('applied', 0., 1.02)
    validator.capture_zeros(case)
    assert case['first_zero']['safe']['source_sim_s'] == 1.01
    assert case['excluded_pre_injection_source_zero']['safe']['source_sim_s'] == .99
    assert case['first_zero']['applied'] is None
    assert case['first_zero_receipt_only']['applied']['value'] == 0.


def test_source_audit_error_is_recorded_for_report_instead_of_raising(monkeypatch):
    validator, _ = fake_validator(monkeypatch)
    validator.check_runtime_sources = lambda: (_ for _ in ()).throw(ValueError('bad manifest'))
    assert not validator.audit_runtime_sources_safely()
    assert 'bad manifest' in validator.report['runtime_source_error']


def test_owned_session_survivor_fails_cleanup_without_killing_it(monkeypatch, tmp_path):
    report = {}
    owned = validation.OwnedProcesses(tmp_path, report)
    saved = dict(pid=42, start_ticks=123, group=10, session=10, parent=10, state='S')
    owned.session_members[(42, 123)] = saved
    monkeypatch.setattr(validation, 'proc_identity', lambda pid: saved.copy())
    touched = []
    monkeypatch.setattr(validation.os, 'kill', lambda *args: touched.append(args))
    assert not owned.cleanup()
    assert report['remaining_session_members'] == [saved]
    assert not touched


def test_final_graph_wait_records_duplicate_gids_until_exact_single_publisher(monkeypatch):
    validator, _ = fake_validator(monkeypatch)
    states = [dict(valid=False, publishers={'/drive/safe': [dict(node='lidar_safety', gid=[1]),
               dict(node='lidar_safety', gid=[2])]}),
              dict(valid=True, publishers={'/drive/safe': [dict(node='lidar_safety', gid=[2])]})]
    validator.graph_snapshot = lambda: states.pop(0)
    validator.graph_audit(convergence_wall_s=40.)
    audit = validator.report['graph_audits'][0]
    assert audit['passed'] and len(audit['changes']) == 2
    assert len(audit['changes'][0]['graph']['publishers']['/drive/safe']) == 2


def test_graph_duplicate_does_not_pass_by_wait_timeout(monkeypatch):
    validator, _ = fake_validator(monkeypatch)
    validator.graph_snapshot = lambda: dict(valid=False, publishers={'/drive/safe': ['lidar_safety', 'lidar_safety']})
    with pytest.raises(AssertionError, match='exact single expected'):
        validator.graph_audit(convergence_wall_s=.05)
    assert not validator.report['graph_audits'][0]['passed']


@pytest.mark.parametrize('duplicate', [False, True])
def test_graph_snapshot_only_accepts_one_expected_publisher(monkeypatch, duplicate):
    validator, _ = fake_validator(monkeypatch)
    owners = {'/drive': 'local_pursuit', '/drive/safe': 'lidar_safety', '/sim/cmd_vel': 'guarded_sim_actuator'}

    def publishers(topic):
        name = owners.get(topic, 'pipeline_fault_validator')
        result = [SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=[1])]
        if duplicate and topic == '/drive/safe':
            result.append(SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=[2]))
        return result

    def subscriptions(name, namespace):
        topics = {'/scan', '/clock', '/drive_motor/encoder', '/imu/data'}
        topics.add('/camera/color/image_raw' if name == 'local_pursuit' else '/drive')
        return [(topic, ['type']) for topic in topics]

    validator.node = SimpleNamespace(get_publishers_info_by_topic=publishers,
                                    get_subscriber_names_and_types_by_node=subscriptions)
    assert validator.graph_snapshot()['valid'] is not duplicate


def test_reused_session_member_does_not_count_as_owned_survivor(monkeypatch, tmp_path):
    owned = validation.OwnedProcesses(tmp_path, {})
    saved = dict(pid=42, start_ticks=123, group=10, session=10, parent=10, state='S')
    owned.session_members[(42, 123)] = saved
    monkeypatch.setattr(validation, 'proc_identity', lambda pid: {**saved, 'start_ticks': 456})
    assert owned.cleanup()
