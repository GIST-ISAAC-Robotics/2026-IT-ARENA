"""선택/저장/fresh process 경계만 검사. 폐루프 주행이나 추가 허가 증명이 아니다."""
from dataclasses import asdict
import copy
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from arena_autonomy.crop_edge_selection import validate_policy_selection, validate_node_selection
from arena_autonomy.decision_process import make_controller
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.native_lidar_input import input_representation

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'runtime_launch_profile_r401_v1/prepared_v2/expected_header.json'


def header(enabled=True):
    value = json.loads(OLD.read_bytes())
    if enabled is not None:
        value['follow_config']['crop_edge_component_trial'] = enabled
    return value


class Node:
    def __init__(self, **overrides):
        self.params = dict(use_sim_time=True, lidar_input_representation='native_simulation_v1')
        self.params.update(overrides)
        self.hardware_inputs = False
        self.motion = NS(mode='both', verified=True, timing_policy='verified_only')
        self.declared = []

    def has_parameter(self, name):
        return name in self.params

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def declare_parameter(self, name, default):
        assert name not in self.params
        self.declared.append(name)
        self.params[name] = default
        return self.get_parameter(name)


def test_old_header_defaults_and_selected_config_roundtrip():
    old = header(None)
    follow, motion = make_controller(old)
    expected = dict(old['follow_config'], crop_edge_component_trial=False)
    assert asdict(follow.config) == expected
    assert not hasattr(follow.partial_policy, 'crop_edge_component_trial')
    for enabled in (False, True):
        follow, _ = make_controller(header(enabled))
        assert asdict(follow.config) == header(enabled)['follow_config']
        follow.reset()
        assert getattr(follow.partial_policy, 'crop_edge_component_trial', False) is enabled


@pytest.mark.parametrize('value', [0, 1, 'true', None])
def test_flag_requires_real_bool(value):
    with pytest.raises(ValueError, match='must be bool'):
        FollowConfig(**dict(header()['follow_config'], crop_edge_component_trial=value))


@pytest.mark.parametrize('policy,recovery', [('strict', True), ('bounded_local_v3', True), ('bounded_local_v4', False)])
def test_only_recovery_v4_selection(policy, recovery):
    with pytest.raises(ValueError, match='bounded_local_v4 and traffic recovery'):
        validate_policy_selection(True, policy, recovery)
    validate_policy_selection(False, policy, recovery)


@pytest.mark.parametrize('field,value', [('use_sim_time', False), ('hardware_inputs', True),
    ('mode', 'off'), ('verified', False), ('timing_policy', 'host_estimated_v1'),
    ('lidar_input_representation', 'laser_scan'), ('lidar_input_representation', 'native_estimated_v1')])
def test_on_rejects_unselected_environment_but_off_untouched(field, value):
    node = Node()
    if field in node.params:
        node.params[field] = value
    elif field == 'hardware_inputs':
        node.hardware_inputs = value
    else:
        setattr(node.motion, field, value)
    before = copy.deepcopy(vars(node))
    validate_node_selection(node, False)
    assert vars(node) == before
    with pytest.raises(ValueError, match='verified-only native simulation'):
        validate_node_selection(node, True)


def test_get_or_declare_keeps_selection_and_existing_default():
    selected = Node()
    validate_node_selection(selected, True)
    assert input_representation(selected) == 'native_simulation_v1'
    assert selected.declared == []
    default = Node()
    default.params.pop('lidar_input_representation')
    assert input_representation(default) == 'laser_scan'
    assert input_representation(default) == 'laser_scan'
    assert default.declared == ['lidar_input_representation']


def test_original_partial_bytes_and_no_new_current_algorithm():
    import arena_autonomy.partial_observation as partial
    assert hashlib.sha256(Path(partial.__file__).read_bytes()).hexdigest() == (
        '576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425')
    # 현재 재검사의 helper나 evaluator는 staged 변경 대상이 아니다.
    assert not (A/'staged/src/arena_autonomy/arena_autonomy/race_runtime_policy').exists()


def native_scan(frame='laser_frame', first=.2, missing=True):
    from arena_autonomy.measurement_scan import MeasurementScan
    degrees = [-179.8 + i*.72 for i in range(500)]
    degrees[388:395] = [99.4, 100.2, 101., 101.7, 102.4, 103.2, 104.]
    return MeasurementScan(tuple(math.inf if missing and 99.4 <= a <= 104. else .57 for a in degrees),
        tuple(math.radians(a) for a in degrees), tuple(first+i*.08/499 for i in range(500)),
        source_frame_id=frame, source_epoch=7, time_basis='simulation_acquisition')


def test_independent_policy_state_and_raw_snapshot_roundtrip(tmp_path):
    from arena_autonomy.lidar_motion import MotionHistory
    from arena_autonomy.scan_input import project_scan
    from arena_autonomy.follow_trace import snapshot_call, restore_call, freeze, FollowTrace
    one, motion = make_controller(header()); two, _ = make_controller(header())
    assert one.partial_policy is not two.partial_policy
    assert one.partial_policy.anchor_probe is not two.partial_policy.anchor_probe
    history = MotionHistory(motion)
    for series in (history.wheels, history.gyro):
        series.times = [i*.005 for i in range(101)]; series.values = [0.]*101
    scan = native_scan(motion.lidar_frame_id)
    snapshot = snapshot_call(scan, history, .3, 11, 0., 'left', 0., .02, True, 0.)
    info, restored, new_history = restore_call(snapshot, motion)
    assert restored == scan and info['epoch'] == 11
    assert restored.source_epoch == 7 and restored.time_basis == 'simulation_acquisition'
    assert new_history.scan_time_policy == 'verified_only'
    assert snapshot_call(restored, new_history, .3, 11, 0., 'left', 0., .02, True, 0.) == snapshot
    points, _, _ = project_scan(scan, history, .3, timing_verified=True)
    cfg = one.config
    kwargs = dict(wheelbase=cfg.wheelbase_m, length=cfg.length_m, width=cfg.width_m)
    warm = native_scan(motion.lidar_frame_id, first=.1, missing=False)
    warm_points, _, _ = project_scan(warm, history, .2, timing_verified=True)
    one.partial_policy.evaluate(warm, warm_points, history, .2, 0., **kwargs)
    assert two.partial_policy.previous is None and two.partial_policy.last_source is None
    result = one.partial_policy.evaluate(scan, points, history, .3, 0., **kwargs)
    assert 'crop_edge_component' in result
    two.partial_policy.evaluate(warm, warm_points, history, .2, 0., **kwargs)
    second = two.partial_policy.evaluate(scan, points, history, .3, 0., **kwargs)
    assert freeze(result) == freeze(second)
    one.partial_policy.reset()
    assert one.partial_policy.previous is None and two.partial_policy.previous is not None
    assert one.partial_policy.crop_edge_component_trial is True
    trace = FollowTrace(str(tmp_path/'trace.jsonl'), motion, cfg)
    trace.record_call(scan, history, .3, 11, 0., 'left', 0., .02, True, 0., [0., 0., result])
    trace.close()
    rows = [json.loads(line) for line in (tmp_path/'trace.jsonl').read_text().splitlines()]
    assert rows[0]['data']['follow_config']['crop_edge_component_trial'] is True
    assert rows[1]['data']['input'] == snapshot


def spawned_factory(stage, connection):
    from arena_autonomy.race_runtime_policy.evaluator import evaluator_factory
    request = header()
    request.update(runtime_policy='cad_runtime_r377_v1', runtime_time_policy='verified_only',
                   numeric_execution='native_v1')
    evaluator = evaluator_factory(stage, request)
    try:
        selected = evaluator.follow.partial_policy
        evaluator.reset('WAIT_GREEN')
        connection.send(dict(stage=stage, configured=evaluator.follow.config.crop_edge_component_trial,
            selected=selected.crop_edge_component_trial, reset=selected.previous is None,
            config=asdict(evaluator.follow.config)))
    finally:
        evaluator.close()
        connection.close()


@pytest.mark.parametrize('stage', ['plan', 'current'])
def test_fresh_native_factory_keeps_selection_and_reset(stage):
    context = mp.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=spawned_factory, args=(stage, child))
    process.start(); child.close()
    try:
        assert parent.poll(30.), 'fresh child did not report selection'
        result = parent.recv()
        assert result == dict(stage=stage, configured=True, selected=True, reset=True,
                              config=header()['follow_config'])
        process.join(5.)
        assert not process.is_alive() and process.exitcode == 0
    finally:
        parent.close()
        if process.is_alive():
            process.terminate(); process.join(3.)
