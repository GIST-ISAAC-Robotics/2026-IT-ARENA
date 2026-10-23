"""CameraInfo 한 표본의 publish_raw→raw callback 바이트 차이 위치를 격리 측정한다."""
import hashlib
import json
import os
from pathlib import Path
import time

os.environ.update(ROS_DOMAIN_ID='192', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
for key in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER'):
    os.environ.pop(key, None)
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from validate_sensor_probe import fixture_messages

HERE = Path(__file__).resolve().parent
assert not (HERE/'report.json').exists()
rclpy.init()
node = rclpy.create_node('camera_info_wire_probe')
sent = bytes(serialize_message(fixture_messages(1, True)['/camera/color/camera_info']))
received = []
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
publisher = node.create_publisher(CameraInfo, '/camera_info_wire_probe', qos)
sub = node.create_subscription(CameraInfo, '/camera_info_wire_probe', lambda data: received.append(bytes(data)), qos, raw=True)
try:
    deadline = time.monotonic()+5
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert publisher.get_subscription_count() == 1
    assert set(node.get_node_names()) == {'camera_info_wire_probe'}
    publisher.publish(sent)
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert len(received) == 1
    got = received[0]
    with (HERE/'sent.cdr').open('xb') as stream:
        stream.write(sent)
    with (HERE/'received.cdr').open('xb') as stream:
        stream.write(got)
    diffs = [dict(offset=i, sent=a, received=b) for i, (a, b) in enumerate(zip(sent, got)) if a != b]
    expected = deserialize_message(sent, CameraInfo)
    actual = deserialize_message(got, CameraInfo)
    report = dict(sent_bytes=len(sent), received_bytes=len(got), differences=diffs,
                  bytes_equal=sent == got, all_message_fields_equal=expected == actual,
                  sent_sha256=hashlib.sha256(sent).hexdigest(), received_sha256=hashlib.sha256(got).hexdigest(),
                  sent_prefix_hex=sent[:16].hex(), received_prefix_hex=got[:16].hex(),
                  sent_suffix_hex=sent[-16:].hex(), received_suffix_hex=got[-16:].hex(),
                  isolated_pc_ros=True, recorder_used=False, hardware_verified=False)
    with (HERE/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report))
finally:
    node.destroy_node()
    rclpy.shutdown()
