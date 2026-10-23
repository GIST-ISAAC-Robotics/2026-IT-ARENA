"""제어기 재시작 구간 진단 기록과 MCU 허가 유지 기대를 ROS 없이 검사한다.

기록은 판정 입력이 아니며, 허가 유지 기대는 기존 다음 단계(재허가 없는 빨강→초록)
전제를 재시작 사례에서 명시한 것이다. 수신 계약·TTL·고장 고정은 바꾸지 않는다.
"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

import validate_pipeline_faults_ros as validation


@pytest.mark.parametrize('role', ['controller', 'safety', 'host', 'mcu', 'sink'])
def test_restart_probes_are_installed_only_in_host_and_mcu_children(role):
    source = validation.child_bootstrap(role)
    compile(source, '<validator child>', 'exec')
    assert ('install_test_mcu_command_probe(ActuationReceiver)' in source) == (role == 'mcu')
    assert ('install_test_host_drive_probe(ActuationHost)' in source) == (role == 'host')
    assert ('install_test_encoder_probe(ActuationReceiver)' in source) == (role == 'mcu')
    assert 'install_test_publish_timing_probe(Publisher)' in source
    assert source.index('RUNTIME_SOURCE_MANIFEST') < source.index('install_test_publish_timing_probe(Publisher)')
    assert source.index('RUNTIME_SOURCE_MANIFEST') < source.index('from ' + validation.ROLES[role][0])
    assert 'scripts/pipeline_restart_probe.py' in validation.FIXTURE_SOURCES


def watched_monitor(monkeypatch):
    wall = [10.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: wall[0])
    monitor = validation.Monitor()
    for key, value in [('applied', 0.), ('sink', {'speed_mps': 0.}), ('mcu', {'target_speed_mps': 0.})]:
        monitor.receive(key, value, 1., None if key == 'applied' else 1.)
    case = dict(sequence=monitor.sequence)
    monitor.begin_zero_watch(case)
    return monitor, wall, case


def test_per_topic_zero_gap_names_the_late_topic_without_changing_bound(monkeypatch):
    monitor, wall, case = watched_monitor(monkeypatch)
    for sim in (1.005, 1.010):
        monitor.receive('applied', 0., sim)
        monitor.receive('mcu', {'target_speed_mps': 0.}, sim, sim)
    monitor.receive('sink', {'speed_mps': 0.}, 1.185, 1.185)
    monitor.receive('applied', 0., 1.185)
    monitor.receive('mcu', {'target_speed_mps': 0.}, 1.185, 1.185)
    monitor.check_zero_watch(wall[0], 1.185)
    observed = case['zero_observation']
    assert observed['maximum_sim_gap_s'] == pytest.approx(.185)
    assert observed['per_topic']['sink']['maximum_sim_gap_s'] == pytest.approx(.185)
    assert observed['per_topic']['sink']['sim_gap_end_received_sim_s'] == 1.185
    assert observed['per_topic']['mcu']['maximum_sim_gap_s'] == pytest.approx(.175)
    monitor.receive('sink', {'speed_mps': 0.}, 1.44, 1.44)
    with pytest.raises(AssertionError, match='receipt gap: sink'):
        monitor.check_zero_watch(wall[0], 1.44)


def bare_validator():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.monitor, validator.t = validation.Monitor(), 30.
    validator.trace, validator.traces = None, []
    validator.children = SimpleNamespace(children=[], vision=[])
    return validator


def test_restart_case_requires_mcu_to_remain_armed_and_traces_the_window(monkeypatch):
    validator = bare_validator()
    calls = []

    def stop_case(name, action, expected, *, no_resume=False):
        calls.append((name, expected, no_resume, validator.trace is not None))
        return {}
    validator.stop_case = stop_case
    validator.red_green = lambda: calls.append(('red_green', validator.trace is not None))
    validator.restart_controller()
    name, expected, no_resume, traced = calls[0]
    assert name == 'controller_restart_green_only_no_start' and no_resume and traced
    assert expected['mcu']({'state': 'ARMED'})
    for state in ('FAULT_LATCHED', 'ESTOP_LATCHED', 'DISARMED', 'ACTIVE'):
        assert not expected['mcu']({'state': state})
    assert expected['autonomy']({'started': False, 'speed_command_mps': 0.})
    assert calls[1] == ('red_green', True)
    assert validator.trace is None and validator.traces[0]['outcome'] == 'passed'
    assert validator.traces[0]['name'] == 'controller_restart_1'


def test_trace_keeps_compact_rows_ticks_and_detects_lost_history(monkeypatch):
    validator = bare_validator()
    samples = []
    validator.sample_processes = lambda threads=False: samples.append(validator.t) or dict(sim_s=validator.t)
    validator.monitor.receive('mcu', {'state': 'ARMED', 'reason': 'zero_target'}, 30., 30.)
    validator.begin_trace('controller_restart_1')
    with pytest.raises(RuntimeError, match='already active'):
        validator.begin_trace('nested')
    for index in range(9):
        validator.t = round(validator.t + .005, 9)
        validator.trace_tick()
        validator.monitor.receive('mcu', {'state': 'FAULT_LATCHED', 'reason': 'command_expired',
                                          'generation': 2, 'path_points': [[0, 0]] * 50}, validator.t, validator.t)
    trace = validator.end_trace('failed: example')
    assert trace['rows_complete'] and len(trace['rows']) == 9 and len(trace['ticks']) == 9
    assert samples == [30.005, 30.025, 30.045]
    assert trace['rows'][0]['value'] == {'state': 'FAULT_LATCHED', 'reason': 'command_expired', 'generation': 2}
    json.dumps(validator.traces, allow_nan=False)
    validator.monitor.rows = validator.monitor.rows.__class__(list(validator.monitor.rows)[3:], maxlen=100)
    validator.begin_trace('controller_restart_2')
    validator.trace['start_sequence'] = 1
    assert not validator.end_trace('passed')['rows_complete']
    assert validator.end_trace('again') is None


def test_probe_log_rows_count_prefixes_and_parse_only_fault_context():
    context = dict(type='test_mcu_fault_context', reason='command_expired')
    slow = dict(type='test_slow_publish', topic='/sim/cmd_vel', duration_s=.3)
    lines = ['MCU_FAULT_CONTEXT=' + json.dumps(context), 'HOST_DRIVE_DECISION={"label": "x"}',
             'HOST_DRIVE_DECISION={broken', 'note MCU_FAULT_CONTEXT={not parsed}', 'MCU_ENCODER_REJECTION={}',
             'SLOW_PUBLISH=' + json.dumps(slow)]
    counts, parsed = validation.probe_log_rows(lines)
    assert counts == {'HOST_DRIVE_DECISION=': 2, 'MCU_FAULT_CONTEXT=': 1, 'MCU_ENCODER_REJECTION=': 1,
                      'SLOW_PUBLISH=': 1}
    assert parsed == {'MCU_FAULT_CONTEXT=': [context], 'SLOW_PUBLISH=': [slow]}


@pytest.mark.skipif(not Path('/proc/self/stat').exists(), reason='Linux /proc required')
def test_proc_sample_reads_own_scheduler_counters_without_signalling():
    sample = validation.proc_sample(os.getpid())
    assert sample['pid'] == os.getpid() and sample['threads'] >= 1
    assert sample['start_ticks'] == validation.proc_identity(os.getpid())['start_ticks']
    assert all(type(sample[key]) is int for key in ('utime_ticks', 'stime_ticks', 'majflt'))
    assert isinstance(validation.pressure_sample(), dict)
    names = {}
    threads = validation.proc_threads(os.getpid(), names)
    assert any(row['tid'] == os.getpid() for row in threads)
    assert all(type(row['run_ns']) is int and row['state'] in 'RSDZTtIWXxKP' for row in threads)
    assert (os.getpid(), os.getpid()) in names
    assert isinstance(validation.shm_listing(), list)


class NodeNameNonExistentError(Exception):
    """rclpy pybind 예외와 같은 이름의 시험 대역."""


def graph_node(missing=(), error=NodeNameNonExistentError, duplicate=None, owner=None,
               absent=None, unresolved=None, extra=None, lacking=None):
    owners = {'/drive': 'local_pursuit', '/drive/safe': 'lidar_safety', '/sim/cmd_vel': 'guarded_sim_actuator'}
    if owner:
        owners[owner[0]] = owner[1]

    def publishers(topic):
        if topic == absent:
            return []
        name = owners.get(topic, 'pipeline_fault_validator')
        if topic == unresolved:
            name = validation.UNKNOWN_NODE_NAME
        result = [SimpleNamespace(node_name=name, node_namespace='/', endpoint_gid=[1])]
        if topic == duplicate:
            result.append(SimpleNamespace(node_name=result[0].node_name, node_namespace='/', endpoint_gid=[2]))
        return result

    def subscriptions(name, namespace):
        if name in missing:
            raise error('cannot get subscriber names and types for nonexistent node')
        topics = {'/scan', '/clock', '/drive_motor/encoder', '/imu/data'}
        topics.add('/camera/color/image_raw' if name == 'local_pursuit' else '/drive')
        if extra and extra[0] == name:
            topics.add(extra[1])
        if lacking and lacking[0] == name:
            topics.discard(lacking[1])
        return [(topic, ['type']) for topic in topics]
    return SimpleNamespace(get_publishers_info_by_topic=publishers,
                           get_subscriber_names_and_types_by_node=subscriptions)


def test_undiscovered_node_name_is_unconverged_not_valid_and_other_errors_still_raise():
    validator = bare_validator()
    validator.node = graph_node(missing=('local_pursuit',))
    audit = validator.graph_snapshot()
    assert not audit['valid'] and audit['missing_nodes'] == ['local_pursuit']
    assert audit['subscriptions']['local_pursuit'] is None and audit['input_checks']['lidar_safety']
    validator.node = graph_node(missing=('lidar_safety',), error=RuntimeError)
    with pytest.raises(RuntimeError):
        validator.graph_snapshot()


def test_startup_graph_waits_only_for_name_discovery_and_records_the_change(monkeypatch):
    validator = bare_validator()
    validator.report = {}
    nodes = [graph_node(missing=('local_pursuit',)), graph_node()]
    pumped = []

    def pump(duration):
        pumped.append(duration)
        validator.node = nodes.pop(0) if nodes else validator.node
    validator.node = nodes.pop(0)
    validator.pump = pump
    validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    observation = validator.report['graph_audits'][0]
    assert observation['passed'] and len(observation['changes']) == 2
    assert observation['policy'] == 'startup_discovery_only'
    assert observation['changes'][0]['graph']['missing_nodes'] == ['local_pursuit']
    assert observation['changes'][0]['graph']['startup_problems'] == dict(
        discovery=['node_name_missing:local_pursuit'], definite=[])
    assert pumped == [.1]
    validator.node = graph_node(missing=('local_pursuit',))
    clock = [0.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: clock[0])
    validator.pump = lambda duration: clock.__setitem__(0, clock[0] + 1.)
    with pytest.raises(AssertionError, match='did not converge'):
        validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    assert clock[0] >= validation.STARTUP_GRAPH_WALL_S


def test_slow_spin_is_recorded_only_inside_trace(monkeypatch):
    from collections import deque
    from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
    wall = [0.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(validation.time, 'sleep', lambda _: None)
    message = lambda: SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()), clock=SimpleNamespace())
    v = bare_validator()
    v.t, v.deadline, v.sleep_s, v.sensor_fault = 1., 100., 0., 'none'
    v.plant, v.history, v.spec = validation.ToyPlant(), deque([(1., 0.)]), MotorEncoderSpec(28, 15., .0325)
    v.data = lambda _: 0.
    v.children = SimpleNamespace(check=lambda: None, children=[], vision=[])
    v.monitor = SimpleNamespace(check_zero_watch=lambda *_: None, sequence=0, rows=deque())
    v.types = {key: message for key in ('clock', 'imu', 'motor', 'scan', 'image')}
    v.publishers = {key: SimpleNamespace(publish=lambda _: None) for key in v.types}
    v.last = dict(motor=-math.inf, scan=-math.inf, image=-math.inf)
    v.images, v.last_counts, v.last_motor_t = False, None, None
    v.sample_processes = lambda threads=False: dict(threads=threads)

    def slow_spin(*_, **__):
        wall[0] += .001
    v.node, v.rclpy = None, SimpleNamespace(spin_once=slow_spin)
    v.tick()
    assert v.trace is None
    v.begin_trace('controller_restart_1')
    v.tick()
    slow = v.trace['slow_spins']
    assert len(slow) == 1 and slow[0]['duration_s'] == pytest.approx(.024)
    assert v.trace['process_samples'] == [dict(threads=True)]


def startup_graph(validator, nodes, monkeypatch):
    """시작 감사 대역: pump 한 번마다 다음 그래프 상태와 벽시계 1 s 진행."""
    clock = [0.]
    monkeypatch.setattr(validation.time, 'monotonic', lambda: clock[0])
    validator.report = {}
    validator.node = nodes.pop(0)

    def pump(duration):
        clock[0] += 1.
        if nodes:
            validator.node = nodes.pop(0)
    validator.pump = pump


@pytest.mark.parametrize('later', ['valid_next', 'duplicate_persists'])
def test_startup_duplicate_fails_immediately_even_if_it_would_disappear(monkeypatch, later):
    """시작 감사의 이전 엄격함 보존: 이미 관측된 중복은 기다리지 않는다."""
    validator = bare_validator()
    following = graph_node() if later == 'valid_next' else graph_node(duplicate='/drive/safe')
    startup_graph(validator, [graph_node(duplicate='/drive/safe'), following], monkeypatch)
    with pytest.raises(AssertionError, match='invalid observed endpoints'):
        validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    observation = validator.report['graph_audits'][0]
    assert not observation['passed'] and len(observation['changes']) == 1
    assert observation['startup_definite_failure'] == ['duplicate_publisher:/drive/safe']
    assert validation.time.monotonic() == 0.


@pytest.mark.parametrize('kwargs,problem', [
    (dict(owner=('/drive/safe', 'local_pursuit')), 'wrong_publisher_owner:/drive/safe'),
    (dict(owner=('/scan', 'intruder')), 'wrong_publisher_owner:/scan'),
    (dict(extra=('lidar_safety', '/camera/color/image_raw')),
     'unexpected_subscription:lidar_safety:/camera/color/image_raw'),
])
def test_startup_wrong_owner_or_unexpected_subscription_fails_immediately(monkeypatch, kwargs, problem):
    validator = bare_validator()
    startup_graph(validator, [graph_node(**kwargs), graph_node()], monkeypatch)
    with pytest.raises(AssertionError, match='invalid observed endpoints'):
        validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    assert validator.report['graph_audits'][0]['startup_definite_failure'] == [problem]
    assert validation.time.monotonic() == 0.


@pytest.mark.parametrize('kwargs,problem', [
    (dict(absent='/drive'), 'publisher_missing:/drive'),
    (dict(unresolved='/drive/safe'), 'publisher_name_unresolved:/drive/safe'),
    (dict(lacking=('local_pursuit', '/camera/color/image_raw')),
     'subscription_missing:local_pursuit:/camera/color/image_raw'),
])
def test_startup_missing_endpoint_or_unresolved_name_waits_then_passes(monkeypatch, kwargs, problem):
    validator = bare_validator()
    startup_graph(validator, [graph_node(**kwargs), graph_node()], monkeypatch)
    validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    observation = validator.report['graph_audits'][0]
    assert observation['passed'] and len(observation['changes']) == 2
    first = observation['changes'][0]['graph']
    assert first['startup_problems'] == dict(discovery=[problem], definite=[]) and not first['valid']


def test_default_final_audit_keeps_converging_through_transient_duplicate(monkeypatch):
    """기존 최종/재시작 감사 수렴 동작은 바꾸지 않는다."""
    validator = bare_validator()
    startup_graph(validator, [graph_node(duplicate='/drive/safe'), graph_node()], monkeypatch)
    validator.graph_audit(convergence_wall_s=40.)
    observation = validator.report['graph_audits'][0]
    assert observation['passed'] and observation['policy'] == 'converge_any_invalid'
    assert len(observation['changes']) == 2
    startup_graph(validator, [graph_node(duplicate='/drive/safe')], monkeypatch)
    with pytest.raises(AssertionError, match='did not converge'):
        validator.graph_audit(convergence_wall_s=3.)
    assert validation.time.monotonic() >= 3.


def test_startup_problem_lists_match_existing_validity():
    validator = bare_validator()
    for kwargs in [dict(), dict(duplicate='/drive'), dict(owner=('/drive', 'lidar_safety')), dict(absent='/clock'),
                   dict(unresolved='/sim/cmd_vel'), dict(extra=('local_pursuit', '/drive')),
                   dict(lacking=('lidar_safety', '/drive')), dict(missing=('lidar_safety',))]:
        validator.node = graph_node(**kwargs)
        audit = validator.graph_snapshot()
        problems = audit['startup_problems']
        assert audit['valid'] is (not problems['discovery'] and not problems['definite']), kwargs


def test_startup_other_lookup_error_raises_without_waiting(monkeypatch):
    validator = bare_validator()
    startup_graph(validator, [graph_node(missing=('lidar_safety',), error=RuntimeError), graph_node()], monkeypatch)
    with pytest.raises(RuntimeError):
        validator.graph_audit(convergence_wall_s=validation.STARTUP_GRAPH_WALL_S, startup=True)
    assert validation.time.monotonic() == 0.
