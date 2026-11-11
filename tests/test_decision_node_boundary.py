"""실제 노드 생성과 STOP/RESET이 별도 계산의 세대를 끊는지 확인한다.

센서·Gazebo·구동기는 실행하지 않는다. 판단 계산은 별도 process이고,
외부 disable/reset은 실제 ROS 노드의 서비스 콜백을 호출한다.
"""
import json
from types import SimpleNamespace as NS
import pytest

rclpy=pytest.importorskip('rclpy')
from std_srvs.srv import SetBool,Trigger
from arena_autonomy.g1_g2_node import G1G2Node
from test_decision_process import fixture


def test_node_disable_reset_and_cleanup_invalidate_real_worker(tmp_path):
    trace=tmp_path/'node.jsonl'
    values=dict(use_sim_time='true',wheelbase_m='.135',lidar_compensation='both',motion_scan_timing_verified='true',
        control_rate_hz='50.0',g1_follow_enabled='true',g1_g2_simulation_trial='true',
        g1_trace_path=str(trace),g1_decision_execution='process_candidate',
        g1_boundary_evidence_mode='corner_distance_candidate',g1_occupancy_timing_mode='per_ray_candidate',
        partial_observation_policy='bounded_local_v4',g1_corridor_fallback_enabled='true',
        g1_parallel_hold_trial='true',g1_pass_release_trial='true',g1_guard_curve_trial='true',
        g1_traffic_recovery_trial='true',g1_traffic_path_recovery_trial='true',vision_execution='inline')
    args=['--ros-args']+[a for k,v in values.items() for a in ('-p',f'{k}:={v}')]
    rclpy.init(args=args)
    node=None
    try:
        node=G1G2Node();worker=node.decision_delivery.worker
        sent=[];node.publisher=NS(publish=sent.append)
        _,sample=fixture()
        assert not node.decision_delivery.offer(sample(.32))
        assert node.decision_delivery.offer(sample(.42))
        generation=worker.generation
        result=node.on_enable(SetBool.Request(data=False),SetBool.Response())
        assert result.success and not node.enabled
        assert worker.generation>generation and node.decision_delivery.active is None
        assert sent[-1].drive.speed==sent[-1].drive.steering_angle==0.
        assert node.last_steering==0.
        generation=worker.generation
        node.on_reset(Trigger.Request(),Trigger.Response())
        assert worker.generation>generation and node.scan is None
        assert node.enabled and not node.signal.started
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
    assert not worker.process.is_alive()
    rows=[json.loads(line) for line in trace.read_text('utf8').splitlines()]
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
    assert any(r['kind']=='reset' and r['data'].get('reason')=='explicit_stop' for r in rows)
