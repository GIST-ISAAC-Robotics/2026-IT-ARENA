"""Extracted bundle + installed ROS only; isolated synthetic input, no vehicle."""
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
target = Path(json.loads((HERE/'isolated_report.json').read_text())['extracted_root'])
manifest = json.loads((HERE/'manifest.json').read_text())
sys.path[:0] = [str(target/'scripts'), str(target/'src/arena_vehicle_interface')]
import sensor_probe_session
assert Path(sensor_probe_session.__file__).resolve().is_relative_to(target)
from arena_vehicle_interface import bag_contract
assert Path(bag_contract.__file__).resolve().is_relative_to(target)

def hashes():
    return {name: hashlib.sha256((target/name).read_bytes()).hexdigest() for name in manifest['files']}

before = hashes()
os.environ.update(ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
for key in ('ROS_STATIC_PEERS','ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
    os.environ.pop(key, None)
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
assert Path(rclpy.__file__).is_relative_to('/opt/ros/jazzy')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('portable_session_fixture')
stop = threading.Event()
errors = []
worker = None
try:
    until = time.monotonic()+1.
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
    assert set(node.get_node_names()) <= {'portable_session_fixture'}
    qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
    imu = node.create_publisher(Imu, '/imu/data', qos)
    scan = node.create_publisher(LaserScan, '/scan', qos)
    def publish():
        try:
            while not stop.is_set():
                stamp = node.get_clock().now().to_msg()
                m = Imu()
                m.header.stamp, m.header.frame_id = stamp, 'fixture_imu'
                m.orientation_covariance[0] = -1.
                m.linear_acceleration.z = 9.81
                s = LaserScan()
                s.header.stamp, s.header.frame_id = stamp, 'fixture_scan'
                s.angle_min, s.angle_max, s.angle_increment = -.1, .1, .1
                s.range_min, s.range_max, s.scan_time = .05, 12., .1
                s.ranges = [1., 1., 1.]
                imu.publish(m)
                scan.publish(s)
                rclpy.spin_once(node, timeout_sec=.05)
        except Exception as error:
            errors.append(repr(error))
    worker = threading.Thread(target=publish)
    worker.start()
    output = target/'artifacts/portable_session'
    code = sensor_probe_session.main(['--output',str(output),'--clock-mode','system',
        '--sensor-topic','/imu/data','--sensor-topic','/scan','--duration','2','--max-raw-gib','.01'])
    result = json.loads((output/'session.json').read_text())
finally:
    stop.set()
    if worker is not None:
        worker.join(timeout=5.)
        assert not worker.is_alive()
    node.destroy_node()
    rclpy.shutdown()

checks = dict(exit_zero=code==0, complete=result['complete'], child_started=result['process_started'],
              child_exited=result['process_exited'], not_forced=not result['forced_termination'],
              integrity=result['record_integrity']['record_integrity_verified'],
              bundle_unchanged=before==hashes(), fixture_closed=not worker.is_alive(), fixture_errors_absent=not errors,
              no_replay_permission=not result['controller_replay_eligible'], no_driving_permission=not result['ready_to_drive'])
report = dict(passed=all(checks.values()), checks=checks, session=result,
              bundle_root=str(target), bundle_sha256=before,
              original_repo_imported=False, hardware_verified=False, jetson_verified=False,
              real_sensor_acquisition_tested=False)
with (HERE/'portable_session_report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(dict(passed=report['passed'],checks=checks)))
raise SystemExit(0 if report['passed'] else 1)
