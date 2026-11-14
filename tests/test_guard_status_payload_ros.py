"""ROS 사용 가능 시 별도 domain에서 실행할 상태 연결 검사. 입력 없는 0명령만 발행한다."""
import json
import time

import pytest

rclpy=pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from arena_autonomy.lidar_safety import LidarSafety


@pytest.mark.parametrize('mode',['full','compact_v1'])
def test_real_node_status_zero_output_and_detail_lifecycle(tmp_path,mode):
    path=tmp_path/'run1_contexts.jsonl'
    rclpy.init(args=['--ros-args','-p','safety_status_detail:='+mode,
        '-p','safety_status_evidence_path:='+str(path)])
    node=listener=executor=None
    try:
        node=LidarSafety();listener=rclpy.create_node('guard_status_contract_listener')
        statuses=[];safe=[]
        listener.create_subscription(String,'/safety/status',lambda m:statuses.append(json.loads(m.data)),10)
        listener.create_subscription(AckermannDriveStamped,'/drive/safe',safe.append,10)
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(listener)
        deadline=time.monotonic()+3.
        while (not statuses or not safe) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        assert statuses and safe
        assert all(row['reason']=='missing_input' and row['safe_speed_mps']==0. for row in statuses)
        assert all(message.drive.speed==0. for message in safe)
        if mode=='full':
            assert node.status_encoder is None and not path.exists()
            assert all('status_detail' not in row for row in statuses)
        else:
            assert node.status_encoder is not None and path.exists()
            assert all(row['status_detail']['mode']=='compact_v1' for row in statuses)
            encoder=node.status_encoder
            encoder({'partial_observation':{'latch_evidence':{'correspondence_context':{'points':[[1.,2.]]}}}})
            encoder.archive.log.queue.join()
            executor.remove_node(node);node.destroy_node();node=None
            original=path.read_bytes()
            assert original and not encoder.archive.log.thread.is_alive()
            node=LidarSafety()  # 같은 path에서 원 증거 보존 + full fallback, 시작 성공
            assert node.status_encoder is None and path.read_bytes()==original
    finally:
        if executor is not None:executor.shutdown(timeout_sec=1.)
        if node is not None:node.destroy_node()
        if listener is not None:listener.destroy_node()
        rclpy.shutdown()
