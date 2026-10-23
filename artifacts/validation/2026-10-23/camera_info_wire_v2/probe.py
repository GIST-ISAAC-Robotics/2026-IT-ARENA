"""별도 프로세스 CameraInfo raw 전달의 바이트/필드 대조. 기록기 없음."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

os.environ.update(ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
for key in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER'):
    os.environ.pop(key, None)
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from validate_sensor_probe import fixture_messages

HERE = Path(__file__).resolve().parent
receiver = '--receiver' in sys.argv
rclpy.init()
node = rclpy.create_node('camera_info_wire_receiver' if receiver else 'camera_info_wire_sender')
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
received = []
child = None
try:
    if receiver:
        sub = node.create_subscription(CameraInfo, '/camera_info_wire_probe', lambda data: received.append(bytes(data)), qos, raw=True)
        deadline = time.monotonic()+10
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(received) == 1
        with (HERE/'received.cdr').open('xb') as stream:
            stream.write(received[0])
    else:
        assert not (HERE/'report.json').exists() and not (HERE/'sent.cdr').exists()
        publisher = node.create_publisher(CameraInfo, '/camera_info_wire_probe', qos)
        with (HERE/'receiver.log').open('x') as log:
            child = subprocess.Popen([sys.executable, '-B', str(Path(__file__).resolve()), '--receiver'],
                                     stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic()+8
            while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert publisher.get_subscription_count() == 1
            assert not (set(node.get_node_names())-{'camera_info_wire_sender', 'camera_info_wire_receiver'})
            sent = bytes(serialize_message(fixture_messages(1, True)['/camera/color/camera_info']))
            with (HERE/'sent.cdr').open('xb') as stream:
                stream.write(sent)
            publisher.publish(sent)
            assert child.wait(timeout=10) == 0
        got = (HERE/'received.cdr').read_bytes()
        diffs = [dict(offset=i, sent=a, received=b) for i, (a, b) in enumerate(zip(sent, got)) if a != b]
        report = dict(sent_bytes=len(sent), received_bytes=len(got), differences=diffs,
                      received_extra_hex=got[len(sent):].hex(), bytes_equal=sent == got,
                      all_message_fields_equal=deserialize_message(sent, CameraInfo) == deserialize_message(got, CameraInfo),
                      sent_sha256=hashlib.sha256(sent).hexdigest(), received_sha256=hashlib.sha256(got).hexdigest(),
                      sent_prefix_hex=sent[:16].hex(), received_prefix_hex=got[:16].hex(),
                      sent_suffix_hex=sent[-16:].hex(), received_suffix_hex=got[-16:].hex(),
                      child_exit=child.returncode, isolated_pc_ros=True, recorder_used=False, hardware_verified=False)
        with (HERE/'report.json').open('x') as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report))
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        child.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
