"""동일 PC의 큰 Image 전달만 대조. 전역 설정/기록기/제어기/실물 변경 없음."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_vehicle_interface')]
from replay_transport import PROFILE_VARIABLES, profile_xml, validate_active
from validate_sensor_probe import fixture_messages

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=('shm_default', 'shm_16m'))
parser.add_argument('--receiver', action='store_true')
args = parser.parse_args()
output = HERE/args.mode
library_path = ROOT/'build/sensor_transport_pc_v1/libprobe.so'
if not args.receiver:
    output.mkdir(exist_ok=False)
    for p in ('scripts/dds_transport_probe.cpp', 'scripts/replay_transport.py',
              'scripts/sensor_probe_delivery.py', 'scripts/validate_sensor_probe.py'):
        dest = output/'sources'/p
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/p, dest)
    if args.mode != 'system':
        with (output/'profile.xml').open('x') as stream:
            stream.write(profile_xml(args.mode))
for key in PROFILE_VARIABLES:
    os.environ.pop(key, None)
os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', SKIP_DEFAULT_XML='1',
                  ROS_DOMAIN_ID='199' if args.mode == 'shm_default' else '200',
                  ROS_LOCALHOST_ONLY='1' if args.mode == 'system' else '0',
                  ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT')
if args.mode != 'system':
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(output/'profile.xml')

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import Image

rclpy.init()
name = 'image_transport_receiver' if args.receiver else 'image_transport_sender'
node = rclpy.create_node(name)
child = None
qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
try:
    assert get_rmw_implementation_identifier() == 'rmw_fastrtps_cpp'
    library = ctypes.CDLL(str(library_path))
    query = library.arena_dds_transport_json
    query.argtypes, query.restype = [ctypes.c_uint], ctypes.c_char_p
    active = json.loads(query(int(os.environ['ROS_DOMAIN_ID'])))
    with (output/(name+'_active.json')).open('x') as stream:
        json.dump(active, stream, indent=2)
    validate_active(active, args.mode)
    if args.receiver:
        received = []
        def callback(payload):
            # Consume only on this isolated receiver. No file write inside callback.
            msg = deserialize_message(payload, Image)
            received.append(dict(fixture_index=int(msg.data[0]), bytes=len(payload),
                                 sha256=hashlib.sha256(payload).hexdigest(),
                                 callback_monotonic_ns=time.monotonic_ns()))
        subscription = node.create_subscription(Image, '/image_transport_probe', callback, qos, raw=True)
        (output/'receiver_ready').touch(exist_ok=False)
        deadline = time.monotonic()+15
        while not (output/'receiver_stop').exists() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert (output/'receiver_stop').exists(), 'parent did not stop bounded receiver'
        with (output/'received.json').open('x') as stream:
            json.dump(received, stream, indent=2)
    else:
        publisher = node.create_publisher(Image, '/image_transport_probe', qos)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.02)
        assert set(node.get_node_names()) == {name}
        with (output/'receiver.log').open('x') as log:
            child = subprocess.Popen([sys.executable, '-B', str(Path(__file__).resolve()), args.mode, '--receiver'],
                                     stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic()+8
            while (not (output/'receiver_ready').exists() or publisher.get_subscription_count() != 1) and time.monotonic() < deadline:
                if child.poll() is not None:
                    raise RuntimeError('receiver stopped before matched endpoints')
                rclpy.spin_once(node, timeout_sec=.05)
            assert (output/'receiver_ready').exists() and publisher.get_subscription_count() == 1
            assert not (set(node.get_node_names())-{'image_transport_sender', 'image_transport_receiver'})
            transmissions = []
            for i in range(10):
                msg = fixture_messages(i, True)['/camera/color/image_raw']
                payload = bytes(serialize_message(msg))
                began = time.monotonic_ns()
                publisher.publish(payload)
                transmissions.append(dict(fixture_index=i, bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest(),
                                          publish_begin_monotonic_ns=began, publish_return_monotonic_ns=time.monotonic_ns()))
                # Wall-time paced diagnostic, not a claimed fixed camera FPS.
                until = time.monotonic()+.2
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.01)
            with (output/'transmissions.json').open('x') as stream:
                json.dump(transmissions, stream, indent=2)
            until = time.monotonic()+1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            (output/'receiver_stop').touch(exist_ok=False)
            assert child.wait(timeout=5) == 0
        received = json.loads((output/'received.json').read_text())
        expected = {t['fixture_index']: t for t in transmissions}
        ids = [r['fixture_index'] for r in received]
        report = dict(diagnostic_completed=True, mode=args.mode, sent=len(transmissions), received=len(received),
                      missing_indices=sorted(set(expected)-set(ids)), duplicate_indices=len(ids)-len(set(ids)),
                      exact_received_payloads=all(r['fixture_index'] in expected and
                        all(r[k] == expected[r['fixture_index']][k] for k in ('bytes','sha256')) for r in received),
                      all_sent_received=len(ids)==10 and len(set(ids))==10 and set(ids)==set(expected),
                      active_settings_checked_both=True, descriptor_not_packet_flow=True, child_exit=child.returncode,
                      recorder_used=False, qos_changed=False, global_settings_changed=False,
                      jetson_verified=False, hardware_verified=False, ready_to_drive=False,
                      library_sha256=hashlib.sha256(library_path.read_bytes()).hexdigest())
        with (output/'report.json').open('x') as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report))
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        child.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
