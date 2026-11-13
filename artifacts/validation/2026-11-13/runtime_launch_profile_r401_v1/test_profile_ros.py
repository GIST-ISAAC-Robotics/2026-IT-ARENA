"""실제 두 노드 parameter→전체 config 대조. 센서/serial/ARM/DRIVE 입력 없음."""
from dataclasses import asdict
import json

from profile_support import PREPARED, load_json, profile, verify_manifest


def test_actual_node_configs_equal_selected_header(tmp_path):
    import rclpy
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    before = verify_manifest()
    prepared = profile(); expected = load_json(PREPARED / 'expected_header.json')
    bindings = load_json(PREPARED / 'bindings.json')
    trace = tmp_path / 'r401_follow.jsonl'
    args = ['--ros-args', '--params-file', str(PREPARED / 'runtime_profile.yaml'),
            '-p', 'g1_trace_path:=' + str(trace)]
    for source, target in bindings['ros_remappings']['local_pursuit']:
        args.extend(['-r', source + ':=' + target])
    rclpy.init(args=args)
    node = guard = worker = vision = None
    processes = []
    try:
        node = G1G2Node(); worker = node.decision_delivery.worker
        vision = node.vision_pipeline.worker.process
        guard = LidarSafety()
        for name, owner in [('local_pursuit', node), ('lidar_safety', guard)]:
            for parameter, value in prepared[name]['ros__parameters'].items():
                assert owner.has_parameter(parameter), (name, parameter)
                actual = owner.get_parameter(parameter).value
                assert type(actual) is type(value) and actual == value, (name, parameter, actual, value)
        assert asdict(node.follow.config) == expected['follow_config']
        assert asdict(node.follow.execution) == expected['execution_config']
        assert asdict(node.motion.history.config) == expected['motion_config']
        assert asdict(guard.motion.history.config) == expected['motion_config']
        assert not node.motion.verified and not guard.motion.verified
        assert node.motion.timing_policy == guard.motion.timing_policy == 'host_estimated_v1'
        assert node.motion.history.config.lidar_frame_id == guard.motion.history.config.lidar_frame_id == 'c1_unmounted_preview'
        assert node.motion.gear_ratio == guard.motion.gear_ratio == 2.5 * 81 / 17
        assert node.rear_blind_half_angle == guard.rear_blind_half_angle
        assert not node.motion.history.wheels.values and not guard.motion.history.wheels.values
        assert node.runtime_dual_selected and node.race is None and not node.signal.started
        assert node.publisher.topic_name == '/drive' and guard.publisher.topic_name == '/drive/safe'
        for name, owner in [('local_pursuit', node), ('lidar_safety', guard)]:
            topics = {subscription.topic_name for subscription in owner.subscriptions}
            for source, target in bindings['ros_remappings'][name]:
                assert target in topics and source not in topics
            assert '/hardware_runtime/imu' in topics and '/drive_motor/encoder' in topics
        assert node.count_publishers('/actuation/tx') == 0
        processes = [worker.plan.process, worker.current.process]
        assert all(process.is_alive() for process in processes)
        assert len({process.pid for process in processes}) == 2 and not hasattr(worker, 'process')
    finally:
        if node is not None:
            node.destroy_node()
        if guard is not None:
            guard.destroy_node()
        rclpy.shutdown()
    assert worker is not None and not any(process.is_alive() for process in processes)
    assert vision is not None and not vision.is_alive() and vision.exitcode == 0
    assert worker.cleanup['forced'] is False and worker.cleanup['alive'] is False and worker.cleanup['exitcode'] == 0
    records = [json.loads(line) for line in trace.read_text(encoding='utf8').splitlines()]
    header = next(row['data'] for row in records if row['kind'] == 'header')
    assert header == expected
    submission = next(row['data'] for row in records if row['kind'] == 'decision_submission_policy')
    assert submission['policy'] == 'runtime_dual_latest_v1'
    assert records[-1]['kind'] == 'footer' and records[-1]['data']['capture_errors'] == 0
    assert verify_manifest() == before
