"""r401 실제 두 노드 구성 검사를 재사용한 ON fixture. 모터/센서/Gazebo 없이 생성만 한다."""
import copy
from dataclasses import asdict
import json
from pathlib import Path

from test_crop_edge_runtime_v2 import A, header, native_scan


def test_actual_two_nodes_and_spawned_children_selected(tmp_path):
    import yaml
    import rclpy
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    from arena_autonomy.follow_trace import snapshot_call, restore_call
    from arena_autonomy.native_lidar_input import decode_measurements
    prepared = yaml.safe_load((A.parent/'runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml').read_text())
    # r401 하드웨어 파일은 그대로 보존. 이 fixture만 명시한 native 시뮬 시간으로 바꾼다.
    for name in ('local_pursuit', 'lidar_safety'):
        prepared[name]['ros__parameters'].update(
            execution_environment='default', use_sim_time=True,
            motion_scan_timing_verified=True, motion_scan_time_policy='verified_only',
            lidar_input_representation='native_simulation_v1', motion_lidar_frame_id='laser_frame',
            g1_crop_edge_component_trial=True)
    profile = tmp_path/'simulation_fixture.yaml'
    profile.write_text(yaml.safe_dump(prepared, sort_keys=False), encoding='utf8')
    expected = header(); expected['motion_config']['lidar_frame_id'] = 'laser_frame'
    trace = tmp_path/'actual_follow.jsonl'
    rclpy.init(args=['--ros-args', '--params-file', str(profile), '-p', 'g1_trace_path:='+str(trace)])
    node = guard = worker = vision = None
    processes = []
    try:
        node = G1G2Node(); worker = node.decision_delivery.worker
        vision = node.vision_pipeline.worker.process
        guard = LidarSafety()
        assert asdict(node.follow.config) == expected['follow_config']
        assert asdict(node.follow.execution) == expected['execution_config']
        for name, owner in [('local_pursuit', node), ('lidar_safety', guard)]:
            for key, value in prepared[name]['ros__parameters'].items():
                assert owner.has_parameter(key), (name, key)
                actual = owner.get_parameter(key).value
                assert type(actual) is type(value) and actual == value, (name, key, actual, value)
            assert asdict(owner.motion.history.config) == expected['motion_config']
            assert owner.motion.verified and owner.motion.timing_policy == 'verified_only'
        assert node.follow.normal.partial_policy.crop_edge_component_trial is True
        assert guard.partial_policy.crop_edge_component_trial is True
        assert node.follow.normal.partial_policy is not guard.partial_policy
        assert node.follow.normal.partial_policy.anchor_probe is not guard.partial_policy.anchor_probe
        assert node.publisher.topic_name == '/drive' and guard.publisher.topic_name == '/drive/safe'
        assert node.runtime_dual_selected and node.count_publishers('/actuation/tx') == 0
        # 선택된 decoder→각 callback→snapshot 경로에서 원 source/frame/time/epoch 보존.
        raw = native_scan()
        payload = asdict(raw)
        payload.update(schema='arena_native_lidar_v1',
            ranges_m=['+inf' if value == float('inf') else value for value in raw.ranges_m])
        decoded = decode_measurements(json.dumps(payload, allow_nan=False),
                                      expected_time_basis='simulation_acquisition')
        assert decoded == raw
        for owner in (node, guard):
            owner.on_scan(decoded)
            for key in ('source_frame_id', 'source_epoch', 'time_basis', 'angles_rad', 'acquired_at_s'):
                assert getattr(owner.scan, key) == getattr(raw, key), key
            snap = snapshot_call(owner.scan, owner.motion.history, .3, 17, 0., 'left', 0., .02, True,
                                 owner.rear_blind_half_angle)
            info, restored, _ = restore_call(snap, owner.motion.history.config)
            assert info['epoch'] == 17 and restored == owner.scan
            assert not owner.motion.history.wheels.values
        assert guard.partial_policy.previous is None
        node.follow.normal.partial_policy.latched = True
        assert guard.partial_policy.latched is False
        node.follow.reset()
        assert node.follow.normal.partial_policy.crop_edge_component_trial is True
        processes = [worker.plan.process, worker.current.process]
        assert len({process.pid for process in processes}) == 2
        assert all(process.is_alive() for process in processes)
        # spin/control/worker.offer를 호출하지 않으며 양수/ARM 명령을 만들지 않는다.
    finally:
        if node is not None:
            node.destroy_node()
        if guard is not None:
            guard.destroy_node()
        rclpy.shutdown()
    assert worker is not None and not any(process.is_alive() for process in processes)
    assert vision is not None and not vision.is_alive() and vision.exitcode == 0
    assert worker.cleanup['forced'] is False and worker.cleanup['alive'] is False
    assert worker.cleanup['exitcode'] == 0
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    assert next(row['data'] for row in rows if row['kind'] == 'header') == expected
    assert next(row['data'] for row in rows if row['kind'] == 'decision_submission_policy')['policy'] == 'runtime_dual_latest_v1'
    assert rows[-1]['kind'] == 'footer' and rows[-1]['data']['capture_errors'] == 0
