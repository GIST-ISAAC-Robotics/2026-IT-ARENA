"""장치 없이 실제 ROS 노드의 두 계산 자식과 STOP/RESET/종료를 확인한다."""
import json
from pathlib import Path
import pytest

rclpy=pytest.importorskip('rclpy')
from std_srvs.srv import SetBool,Trigger
from arena_autonomy.g1_g2_node import G1G2Node
from arena_autonomy.lidar_safety import LidarSafety


def test_runtime_selected_before_spawn_and_shutdown(tmp_path):
    trace=tmp_path/'runtime.jsonl'
    args=['--ros-args','--params-file',str(Path.cwd()/'config/hardware/g1_input_shadow_v1.yaml')]
    values=dict(execution_environment='hardware_runtime_candidate_v1',use_sim_time='false',
        motion_scan_time_policy='host_estimated_v1',lidar_compensation='both',
        lidar_input_representation='native_estimated_v1',g1_runtime_policy='cad_runtime_r377_v1',
        g1_numeric_execution='native_v1',g1_expiry_route_reference_trial='true',g1_junction_continuity='true',
        g1_trace_path=str(trace),partial_observation_policy='bounded_local_v4')
    args += [v for key,value in values.items() for v in ('-p',key+':='+value)]
    rclpy.init(args=args);node=guard=None;worker=None
    try:
        node=G1G2Node();worker=node.decision_delivery.worker
        guard=LidarSafety()
        assert node.runtime_dual_selected and node.publisher.topic_name=='/drive'
        assert guard.publisher.topic_name=='/drive/safe'
        assert not node.motion.verified and node.motion.timing_policy=='host_estimated_v1'
        assert not node.motion.history.wheels.values
        assert node.count_publishers('/actuation/tx')==0
        assert not hasattr(worker,'process')
        processes=[worker.plan.process,worker.current.process]
        assert all(p.is_alive() for p in processes) and processes[0].pid!=processes[1].pid
        before=worker.generation
        response=node.on_enable(SetBool.Request(data=False),SetBool.Response())
        assert response.success and not node.enabled and worker.generation>before
        assert worker.retained is None and worker.ready is None
        before=worker.generation
        node.on_reset(Trigger.Request(),Trigger.Response())
        assert worker.generation>before and node.scan is None and node.enabled and not node.signal.started
    finally:
        if node is not None:node.destroy_node()
        if guard is not None:guard.destroy_node()
        rclpy.shutdown()
    assert worker is not None and not any(p.is_alive() for p in processes)
    assert not worker.cleanup['forced'] and not worker.cleanup['alive'] and worker.cleanup['exitcode']==0
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    policy=next(row['data'] for row in rows if row['kind']=='runtime_factory_policy')
    assert policy['motion_time_policy']=='host_estimated_v1'
    assert policy['source_timestamps_preserved'] and not policy['motion_timing_verified']
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
