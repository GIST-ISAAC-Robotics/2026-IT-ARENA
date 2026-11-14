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


@pytest.mark.parametrize('recheck,submission,guard_policy,body_gap,expiry', [
    ('off','original_v1','original_v1',False,False),
    ('selected_v1','original_v1','original_v1',False,False),
    ('selected_v1','fresh_native_v1','original_v1',False,False),
    ('off','original_v1','selected_r181',False,False),
    ('selected_v1','original_v1','selected_r181',False,False),
    ('selected_v1','fresh_native_v1','selected_r181',False,False),
    ('selected_v1','original_v1','selected_r181',True,False),
    ('selected_v1','fresh_native_v1','selected_r181',True,False),
    ('selected_v1','fresh_native_v1','selected_r181',True,True)])
def test_node_disable_reset_and_cleanup_invalidate_real_worker(tmp_path, recheck, submission, guard_policy, body_gap, expiry, recheck_execution='parent'):
    trace=tmp_path/'node.jsonl'
    values=dict(use_sim_time='true',wheelbase_m='.135',lidar_compensation='both',motion_scan_timing_verified='true',
        control_rate_hz='50.0',g1_follow_enabled='true',g1_g2_simulation_trial='true',
        g1_trace_path=str(trace),g1_decision_execution='process_candidate',
        # ROS CLI는 따옴표 없는 off를 YAML bool False로 해석한다.
        g1_current_path_recheck='"'+recheck+'"',
        g1_submission_policy=submission,
        g1_current_recheck_execution=recheck_execution,
        g1_guard_observation_policy=guard_policy,
        g1_boundary_evidence_mode='corner_distance_candidate',g1_occupancy_timing_mode='per_ray_candidate',
        partial_observation_policy='bounded_local_v4',g1_corridor_fallback_enabled='true',
        g1_parallel_hold_trial='true',g1_pass_release_trial='true',g1_guard_curve_trial='true',
        g1_traffic_recovery_trial='true',g1_traffic_path_recovery_trial='true',vision_execution='inline')
    if submission == 'fresh_native_v1':
        values['lidar_input_representation'] = 'native_simulation_v1'
    values['g1_released_body_gap_trial'] = str(body_gap).lower()
    values['g1_expiry_route_reference_trial'] = str(expiry).lower()
    if expiry:
        values['g1_junction_continuity'] = 'true'
    args=['--ros-args']+[a for k,v in values.items() for a in ('-p',f'{k}:={v}')]
    rclpy.init(args=args)
    node=None
    try:
        node=G1G2Node();worker=node.decision_delivery.worker
        from arena_autonomy.fresh_submission import FreshStartDelivery
        assert isinstance(node.decision_delivery,FreshStartDelivery) == (submission == 'fresh_native_v1')
        assert (node.current_path_recheck is not None) == (recheck == 'selected_v1')
        assert node.follow.config.guard_observation_policy == guard_policy
        assert node.follow.config.released_body_gap_trial is body_gap
        assert node.follow.config.expiry_route_reference_trial is expiry
        if expiry:
            assert node.decision_delivery.retain_expired_reference is True
            worker.mark_source_expiry()
            assert worker.expiry_generation == worker.generation
        if recheck == 'selected_v1' and guard_policy == 'selected_r181':
            from arena_autonomy import guard_observation_policy
            assert node.current_path_recheck is guard_observation_policy
        sent=[];node.publisher=NS(publish=sent.append)
        _,sample=fixture()
        assert not node.decision_delivery.offer(sample(.32))
        assert node.decision_delivery.offer(sample(.42))
        generation=worker.generation
        result=node.on_enable(SetBool.Request(data=False),SetBool.Response())
        assert result.success and not node.enabled
        assert worker.generation>generation and node.decision_delivery.active is None
        assert worker.expiry_generation is None
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
    assert node.decision_notice is None and node.decision_guard is None
    rows=[json.loads(line) for line in trace.read_text('utf8').splitlines()]
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
    assert rows[0]['data']['follow_config']['guard_observation_policy'] == guard_policy
    assert rows[0]['data']['follow_config']['released_body_gap_trial'] is body_gap
    assert rows[0]['data']['follow_config']['expiry_route_reference_trial'] is expiry
    expiry_policies=[r for r in rows if r['kind']=='expiry_reference_policy']
    assert len(expiry_policies)==int(expiry)
    if expiry:
        assert expiry_policies[0]['data']['command_retained'] is False
    assert any(r['kind']=='reset' and r['data'].get('reason')=='explicit_stop' for r in rows)
    policies = [r for r in rows if r['kind'] == 'current_path_recheck_policy']
    assert len(policies) == (1 if recheck == 'selected_v1' else 0)
    if policies:
        assert policies[0]['data']['evaluation_delay_injection'] is False
    submissions = [r for r in rows if r['kind'] == 'decision_submission_policy']
    assert len(submissions) == (1 if submission == 'fresh_native_v1' else 0)
    if submissions:
        assert submissions[0]['data']['evaluation_delay_injection'] is False
        assert submissions[0]['data']['completion_window_s'] == .025


def test_two_stage_node_disable_reset_and_cleanup(tmp_path):
    test_node_disable_reset_and_cleanup_invalidate_real_worker(tmp_path, 'selected_v1',
        'fresh_native_v1', 'selected_r181', True, True, 'worker_event_v1')
