"""실제 ROS 노드/두 spawn 자식의 STOP·RESET·종료 연결. 차량은 실행하지 않는다."""
import json
import hashlib
import importlib.util
import sys
from pathlib import Path
import time

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A))

import pytest
rclpy = pytest.importorskip('rclpy')
from std_srvs.srv import SetBool, Trigger
from ackermann_msgs.msg import AckermannDriveStamped
from arena_autonomy.g1_g2_node import G1G2Node
from test_decision_process import fixture

spec = importlib.util.spec_from_file_location('r365_own_node_entry', A/'node_entry_draft.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
install = entry.install
assert Path(entry.__file__).resolve() == A/'node_entry_draft.py'


def test_real_node_stop_reset_terminate_both_workers(tmp_path, monkeypatch):
    trace = tmp_path/'node.jsonl'
    values = dict(use_sim_time='true', wheelbase_m='.128', vehicle_length_m='.207', vehicle_width_m='.15', motion_rear_axle_x_m='-.064', lidar_compensation='both',
        motion_scan_timing_verified='true', control_rate_hz='50.0', g1_follow_enabled='true',
        g1_g2_simulation_trial='true', g1_trace_path=str(trace), g1_decision_execution='process_candidate',
        g1_current_path_recheck='selected_v1', g1_submission_policy='fresh_native_v1',
        g1_current_recheck_execution='worker_event_v1', g1_numeric_execution='native_v1',
        g1_guard_observation_policy='selected_r181', g1_boundary_evidence_mode='corner_distance_candidate',
        g1_occupancy_timing_mode='per_ray_candidate', partial_observation_policy='bounded_local_v4',
        g1_corridor_fallback_enabled='true', g1_parallel_hold_trial='true', g1_pass_release_trial='true',
        g1_guard_curve_trial='true', g1_traffic_recovery_trial='true', g1_traffic_path_recovery_trial='true',
        g1_released_body_gap_trial='true', g1_expiry_route_reference_trial='true',
        g1_junction_continuity='true', lidar_input_representation='native_simulation_v1',
        vision_execution='inline')
    args = ['--ros-args']+[a for k, v in values.items() for a in ('-p', f'{k}:={v}')]
    args += ['-r', '/drive:=/r365_test/drive', '-r', '/autonomy/status:=/r365_test/status']
    restore = install(A/'profile.draft.json')
    rclpy.init(args=args)
    node = None
    try:
        node = G1G2Node()
        assert node.follow.config.wheelbase_m == .128
        assert node.follow.config.length_m == .207
        assert node.follow.config.width_m == .15
        assert node.motion.history.config.rear_axle_x_m == -.064
        node.get_clock().set_ros_time_override(rclpy.time.Time(nanoseconds=420000000))
        d, w = node.decision_delivery, node.decision_delivery.worker
        children = [w.plan.process, w.current.process]
        assert all(p.is_alive() for p in children)
        assert node.publisher.topic_name == '/r365_test/drive'
        sent = []
        node.create_subscription(AckermannDriveStamped, '/r365_test/drive', sent.append, 10)
        _, sample = fixture()
        assert not d.offer(sample(.32))
        assert d.offer(sample(.42))
        generation = w.generation
        result = node.on_enable(SetBool.Request(data=False), SetBool.Response())
        assert result.success and not node.enabled and w.generation > generation
        assert w.retained is w.ready is d.pending_publication is None
        until = time.monotonic()+3.
        while time.monotonic() < until and not sent:
            rclpy.spin_once(node, timeout_sec=.02)
        assert sent and all(m.drive.speed == m.drive.steering_angle == 0. for m in sent)
        generation = w.generation
        result = node.on_reset(Trigger.Request(), Trigger.Response())
        assert result.success and w.generation > generation
        assert node.scan is None and not node.signal.started
        assert w.retained is w.ready is d.pending_publication is None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        restore()
    assert all(not p.is_alive() and p.exitcode == 0 for p in children)
    assert not w.cleanup['forced'] and w.cleanup['exitcode'] == 0
    rows = [json.loads(s) for s in trace.read_text().splitlines()]
    policies = [r['data'] for r in rows if r['kind']=='runtime_factory_policy']
    assert len(policies)==1 and policies[0]['backend']=='runtime_dual_r362'
    assert policies[0]['policy']=='r353' and policies[0]['hardware_enabled'] is False
    assert any(r['kind']=='reset' and r['data'].get('reason')=='explicit_stop' for r in rows)
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
