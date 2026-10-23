"""실제 ROS 기록의 계획 종료와 주입된 close 멎음을 분리한다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import threading
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan

from sensor_probe_session import main, supervise
from validate_sensor_probe import fixture_messages

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sources = ['scripts/sensor_probe_session.py','scripts/record_sensor_bag.py','scripts/recording_guard.py',
    'scripts/rosbag_writer_compat.py','scripts/audit_sensor_probe.py','scripts/audit_sensor_bag.py',
    'scripts/sensor_recording_profile.py','scripts/validate_sensor_probe.py','scripts/sensor_probe_delivery.py',
    'tests/test_sensor_probe_session.py','tests/fixtures/sensor_probe_stalled_recorder.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py']
before = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
for p in sources:
    dest = HERE/'sources'/p
    dest.parent.mkdir(parents=True,exist_ok=True)
    with dest.open('xb') as out,(ROOT/p).open('rb') as src:
        shutil.copyfileobj(src,out)
os.environ.update(ROS_DOMAIN_ID='192',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
for key in ('ROS_STATIC_PEERS','ROS_DISCOVERY_SERVER','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
    os.environ.pop(key,None)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('sensor_probe_session_fixture')
until = time.monotonic()+.5
while time.monotonic()<until:
    rclpy.spin_once(node,timeout_sec=.05)
assert set(node.get_node_names()) <= {'sensor_probe_session_fixture'}
qos = QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT)
pubs = {'/imu/data':node.create_publisher(Imu,'/imu/data',qos),
        '/scan':node.create_publisher(LaserScan,'/scan',qos)}
stop = threading.Event()
errors = []
def publish():
    try:
        index=0
        while not stop.is_set():
            for topic,msg in fixture_messages(index).items():
                pubs[topic].publish(msg)
            index+=1
            rclpy.spin_once(node,timeout_sec=.02)
    except Exception as error:
        errors.append(repr(error))
worker=threading.Thread(target=publish)
worker.start()
try:
    normal = HERE/'normal'
    exit_code=main(['--output',str(normal),'--clock-mode','system','--sensor-topic','/imu/data',
                   '--sensor-topic','/scan','--duration','2','--max-raw-gib','.01'])
    normal_result=json.loads((normal/'session.json').read_text())
    hung=HERE/'stalled'
    hung.mkdir()
    recording=hung/'sensor_recording'
    event=hung/'entered_close.json'
    os.environ['ARENA_STALLED_CLOSE_EVENT']=str(event)
    command=[sys.executable,str(ROOT/'tests/fixtures/sensor_probe_stalled_recorder.py'),
        '--output',str(recording),'--profile','sensor_probe','--clock-mode','system',
        '--sensor-topic','/imu/data','--sensor-topic','/scan','--max-wall-seconds','15','--max-raw-gib','.01']
    with (hung/'recorder.log').open('x') as log:
        stalled_result=supervise(command,recording/'ready.json',log,duration=1.,startup_timeout=30.,
                                 stop_timeout=.5,term_timeout=.3,kill_timeout=3.)
    with (hung/'supervision.json').open('x') as stream:
        json.dump(stalled_result,stream,indent=2)
    checks=dict(normal_exit=exit_code==0 and normal_result['complete'],
                normal_not_forced=not normal_result['forced_termination'],
                normal_integrity=normal_result['record_integrity']['record_integrity_verified'],
                actual_close_stall_entered=event.is_file(),
                stalled_process_exited=stalled_result['process_exited'],
                stalled_failed=not stalled_result['lifecycle_complete'],
                stalled_killed=stalled_result['exit_code']==-9 and stalled_result['forced_termination'],
                stalled_no_complete_manifest=not (recording/'manifest.json').exists())
finally:
    stop.set()
    worker.join(timeout=5.)
    assert not worker.is_alive()
    node.destroy_node()
    rclpy.shutdown()
after={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
checks.update(source_unchanged=before==after,fixture_thread_closed=not worker.is_alive(),fixture_no_errors=not errors)
report=dict(passed=all(checks.values()),checks=checks,source_sha256=before,
            normal=normal_result,stalled=stalled_result,physical_disk_failure_reproduced=False,
            real_75_second_grace_tested=False,hardware_verified=False,ready_to_drive=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report,stream,indent=2)
print(json.dumps(dict(passed=report['passed'],checks=checks)))
raise SystemExit(0 if report['passed'] else 1)
