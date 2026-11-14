"""명시 저속 공백의 실제 노드/독립 보호 연결과 STOP. 센서·구동기 없음."""
import json
import pytest
rclpy=pytest.importorskip('rclpy')
from std_srvs.srv import SetBool
from arena_autonomy.g1_g2_node import G1G2Node
from arena_autonomy.lidar_safety import LidarSafety


def test_explicit_traffic_option_keeps_independent_guard_and_stop(tmp_path):
    trace=tmp_path/'trace.jsonl'
    values=dict(use_sim_time='true',wheelbase_m='.135',lidar_compensation='both',motion_scan_timing_verified='true',
        control_rate_hz='50.0',g1_follow_enabled='true',g1_g2_simulation_trial='true',
        g1_trace_path=str(trace),g1_decision_execution='process_candidate',
        g1_current_path_recheck='"selected_v1"',g1_submission_policy='fresh_native_v1',
        lidar_input_representation='native_simulation_v1',
        g1_boundary_evidence_mode='corner_distance_candidate',g1_occupancy_timing_mode='per_ray_candidate',
        partial_observation_policy='bounded_local_v4',g1_parallel_hold_trial='true',
        g1_pass_release_trial='true',g1_guard_curve_trial='true',g1_traffic_recovery_trial='true',
        g1_traffic_path_recovery_trial='true',g1_corridor_fallback_enabled='true',
        g1_oblique_gap_trial='true',vision_execution='inline')
    rclpy.init(args=['--ros-args']+[item for k,v in values.items() for item in ('-p',f'{k}:={v}')])
    node=guard=None
    try:
        node=G1G2Node();guard=LidarSafety()
        assert node.follow.config.oblique_gap_trial
        assert not node.follow.normal.partial_policy.oblique_metric_enabled
        assert guard.partial_policy.oblique_metric_enabled
        assert node.publisher.topic_name=='/drive'
        assert guard.publisher.topic_name=='/drive/safe'
        generation=node.decision_delivery.worker.generation
        answer=node.on_enable(SetBool.Request(data=False),SetBool.Response())
        assert answer.success and not node.enabled
        assert node.decision_delivery.worker.generation>generation
        assert node.last_speed==node.last_steering==0.
    finally:
        if guard is not None:guard.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    assert rows[0]['data']['follow_config']['oblique_gap_trial']
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
