"""실제 ROS 시계·공통 노드의 관측 전용 연결. 합성 입력이며 실센서 교정이 아니다."""
from pathlib import Path
import json
import math
import time

import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from arena_autonomy.g1_g2_node import G1G2Node

ROOT=Path(__file__).resolve().parents[1]
PROFILE=ROOT/'config/hardware/g1_input_shadow_v1.yaml'


def test_real_clock_shadow_keeps_unverified_input_and_no_actuator_output(tmp_path):
    trace=tmp_path/'shadow.jsonl'
    rclpy.init(args=['--ros-args','--params-file',str(PROFILE),'-p','g1_trace_path:='+str(trace)])
    node=peer=None
    try:
        node=G1G2Node();peer=Node('shadow_test_peer')
        assert node.hardware_shadow and not node.motion.verified
        assert node.now_s()>1_000_000_000
        assert node.publisher.topic_name=='/hardware_preview/drive'
        assert node.count_publishers('/drive')==node.count_publishers('/drive/safe')==0
        assert node.count_publishers('/actuation/tx')==0
        outputs=[];states=[]
        peer.create_subscription(AckermannDriveStamped,'/hardware_preview/drive',outputs.append,10)
        peer.create_subscription(String,'/hardware_preview/autonomy/status',lambda m:states.append(json.loads(m.data)),10)
        pub=peer.create_publisher(String,'/scan/native',qos_profile_sensor_data)
        until=time.monotonic()+3
        while pub.get_subscription_count()==0:
            rclpy.spin_once(peer,timeout_sec=.01)
            assert time.monotonic()<until
        now=node.now_s()
        raw=dict(schema='arena_native_lidar_v1',ranges_m=[2.]*40,
            angles_rad=[math.pi-i*math.tau/40 for i in range(40)],
            acquired_at_s=[now-.1+i*.002 for i in range(40)],
            source_frame_id='laser_frame',source_epoch=17,range_min_m=.05,range_max_m=12.,
            time_basis='unverified_estimate')
        pub.publish(String(data=json.dumps(raw)))
        until=time.monotonic()+2
        while node.scan is None:
            rclpy.spin_once(node,timeout_sec=.01)
            assert time.monotonic()<until
        assert node.scan.time_basis=='unverified_estimate' and node.scan.source_epoch==17
        # 입력 거절에 도달시키기 위한 시험 장치. 실제 신호 검출/출발을 뜻하지 않는다.
        node.signal.started=True;node.rgb_stamp=node.now_s();node.image_wall_time=time.monotonic()
        node.control()
        assert node.last_status=='MOTION_STOP'
        assert node.motion.last_meta['reason']=='scan_timing_unverified'
        assert not node.motion.history.wheels.values
        until=time.monotonic()+.15
        while time.monotonic()<until:rclpy.spin_once(peer,timeout_sec=.01)
        assert outputs and all(m.drive.speed==0 for m in outputs)
        assert any(s.get('drive_output') is False and s.get('motion_timing_verified') is False for s in states)
        generation=node.decision_delivery.worker.generation
        assert node.on_enable(SetBool.Request(data=False),SetBool.Response()).success
        assert node.decision_delivery.worker.generation>generation
        node.on_reset(Trigger.Request(),Trigger.Response())
        assert node.scan is None and not node.signal.started
        process=node.decision_delivery.worker.process
    finally:
        if node is not None:node.destroy_node()
        if peer is not None:peer.destroy_node()
        rclpy.shutdown()
    assert not process.is_alive()
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    environment=next(r['data'] for r in rows if r['kind']=='execution_environment')
    assert environment==dict(mode='hardware_shadow_v1',use_sim_time=False,drive_output=False,motion_timing_verified=False)
    assert rows[-1]['kind']=='footer' and rows[-1]['data']['capture_errors']==0
