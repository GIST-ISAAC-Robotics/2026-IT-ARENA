"""별도 실물 운용 선택. 실제 출력·장착 정확도 검증이 아니다."""
from types import SimpleNamespace
import json
from pathlib import Path
import pytest

from arena_autonomy.execution_environment import configure, validate_motion_time, motion_supported


class Params:
    def __init__(self, **values):
        self.values = values
    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.values.setdefault(name, default))
    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


@pytest.mark.parametrize('environment', ['hardware_shadow_v1', 'hardware_runtime_candidate_v1'])
def test_explicit_physical_estimate_remains_unverified(environment):
    node = Params(execution_environment=environment, use_sim_time=False)
    configure(node)
    validate_motion_time(node, 'both', False, 'host_estimated_v1')
    node.motion = SimpleNamespace(mode='both', verified=False, timing_policy='host_estimated_v1')
    assert motion_supported(node) and not node.motion.verified
    assert node.hardware_inputs and node.hardware_shadow == (environment == 'hardware_shadow_v1')


@pytest.mark.parametrize('fault', ['sim_clock', 'verified', 'default_policy', 'no_motion', 'default_environment'])
def test_reject_mixed_or_implicit_runtime_contract(fault):
    node = Params(execution_environment='hardware_runtime_candidate_v1', use_sim_time=False)
    if fault == 'sim_clock': node.values['use_sim_time'] = True
    if fault == 'default_environment': node.values['execution_environment'] = 'default'
    with pytest.raises(ValueError):
        configure(node)
        validate_motion_time(node, 'none' if fault == 'no_motion' else 'both',
                             fault == 'verified',
                             'verified_only' if fault == 'default_policy' else 'host_estimated_v1')


def test_legacy_default_and_shadow_no_claim_are_preserved():
    for environment in ('default', 'hardware_shadow_v1'):
        node = Params(execution_environment=environment, use_sim_time=False)
        configure(node)
        validate_motion_time(node, 'both', False, 'verified_only')


def test_real_ros_runtime_nodes_keep_independent_guard_and_no_physical_output(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    root = Path.cwd()
    profile = root/'config/hardware/g1_input_shadow_v1.yaml'
    trace = tmp_path/'runtime.jsonl'
    rclpy.init(args=['--ros-args', '--params-file', str(profile),
                    '-p', 'execution_environment:=hardware_runtime_candidate_v1',
                    '-p', 'motion_scan_time_policy:=host_estimated_v1',
                    '-p', 'g1_trace_path:='+str(trace),
                    '-p', 'lidar_compensation:=both',
                    '-p', 'lidar_input_representation:=native_estimated_v1',
                    '-p', 'partial_observation_policy:=bounded_local_v4'])
    planner = guard = None
    try:
        planner = G1G2Node()
        guard = LidarSafety()
        assert planner.publisher.topic_name == '/drive'
        assert guard.publisher.topic_name == '/drive/safe'
        assert not planner.motion.verified and not guard.motion.verified
        assert planner.motion.history.scan_time_policy == guard.motion.history.scan_time_policy == 'host_estimated_v1'
        assert guard.partial_policy is not None
        assert not planner.motion.history.wheels.times and not guard.motion.history.wheels.times
        assert planner.count_publishers('/actuation/tx') == 0
        assert planner.count_publishers('/hardware_preview/drive') == 0
        process = planner.decision_delivery.worker.process
    finally:
        if planner is not None: planner.destroy_node()
        if guard is not None: guard.destroy_node()
        rclpy.shutdown()
    assert not process.is_alive()
    records = [json.loads(line) for line in trace.read_text().splitlines()]
    declaration = next(row['data'] for row in records if row['kind'] == 'execution_environment')
    assert declaration['motion_timing_verified'] is False
    assert declaration['physical_actuation_verified'] is False
