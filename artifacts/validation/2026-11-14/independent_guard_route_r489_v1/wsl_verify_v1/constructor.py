"""실제 ROS 생성자 연결만 확인. executor/spin/control 호출 없음."""
import hashlib
import json
import os
from pathlib import Path
import rclpy
from arena_autonomy.lidar_safety import LidarSafety
import arena_autonomy.lidar_safety as source

A=Path(__file__).resolve().parent
(A/'constructor.pid').write_text(str(os.getpid()))
node=None
result=dict(pid=os.getpid(),spin_called=False,control_called=False,drive_publications=0,
    scope='constructor only; no sensor input or executor',source_file=source.__file__,
    source_sha256=hashlib.sha256(Path(source.__file__).read_bytes()).hexdigest())
try:
    rclpy.init(args=['--ros-args','-r','__ns:=/r489_constructor_probe',
        '-p','use_sim_time:=true','-p','lidar_compensation:=both',
        '-p','motion_scan_timing_verified:=true',
        '-p','partial_observation_policy:=bounded_local_v4',
        '-p','g1_oblique_gap_trial:=true','-p','g1_traffic_recovery_trial:=true',
        '-p','g1_guard_observation_policy:=selected_r181'])
    node=LidarSafety()
    result.update(node_policy=node.get_parameter('g1_guard_observation_policy').value,
        command_observation_priority=node.command_observation_priority,
        partial_command_observation_priority=node.partial_policy.command_observation_priority,
        partial_class=type(node.partial_policy).__name__)
    assert result['command_observation_priority'] is True
    assert result['partial_command_observation_priority'] is True
    assert result['partial_class']=='RecoveryPolicy'
    result['passed']=True
finally:
    if node is not None:node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    result['shutdown_complete']=not rclpy.ok()
    (A/'constructor.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
