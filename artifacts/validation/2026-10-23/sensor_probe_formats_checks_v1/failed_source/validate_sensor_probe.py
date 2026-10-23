#!/usr/bin/env python3
"""격리 localhost ROS에서 센서 점검 기록을 검사한다. Gazebo/제어/구동/장치 없음."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import load_manifest, sha256_file
from audit_sensor_probe import audit
from audit_sensor_bag import reader_for
from arena_vehicle_interface.drive_feedback import MOTOR_TOPIC, MOTOR_JOINT, MOTOR_FRAME, MotorEncoderSpec


def fixture_messages(index, full_input=False):
    """합성 직렬화 표본. 카메라 교정·실물 취득 시각/모터 관측을 주장하지 않는다."""
    from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo, JointState
    stamp = (0, 1, 2, 1)[index % 4]
    imu = Imu()
    imu.header.stamp.sec, imu.header.frame_id = stamp, 'imu_fixture'
    imu.angular_velocity.z = index/1000.
    scan = LaserScan()
    scan.header.stamp.sec, scan.header.frame_id = stamp, 'laser_fixture'
    scan.angle_min, scan.angle_max, scan.angle_increment = -.5, .5, .5
    scan.time_increment, scan.scan_time = .01, .1
    scan.range_min, scan.range_max = .05, 12.
    scan.ranges = [1.+index/1000., float('inf'), 2.]
    messages = {'/imu/data': imu, '/scan': scan}
    if full_input:
        image = Image()
        image.header.stamp.sec, image.header.frame_id = stamp, 'camera_fixture_uncalibrated'
        image.width, image.height = 1280, 720
        image.encoding = 'rgb8' if index % 2 == 0 else 'bgr8'
        padding = 0 if index % 2 == 0 else 16
        image.step = image.width*3+padding
        pixels = bytes((index, 37, 191))*image.width + bytes([0xee])*padding
        image.data = pixels*image.height
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        # k[0] stays 0: no guessed calibration is represented as measured intrinsics.
        motor = JointState()
        motor.header.stamp.sec, motor.header.frame_id = stamp, MOTOR_FRAME
        motor.name = [MOTOR_JOINT]
        spec = MotorEncoderSpec(28, 15., .0325)
        motor.position = [spec.motor_angle_from_counts(index-10)]
        motor.velocity = [spec.motor_rad_s_from_counts((0, 1, -1)[index % 3], .01)]
        messages.update({'/camera/color/image_raw': image, '/camera/color/camera_info': info,
                         MOTOR_TOPIC: motor})
    return messages


def trial(root, case, simulation_clock=False, publish_clock=False, regress_clock=False, full_input=False):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    root.mkdir(parents=True, exist_ok=False)
    recording = root/'sensor_recording'
    command = [sys.executable, str(ROOT/'scripts/record_sensor_bag.py'),
               '--output', str(recording), '--max-wall-seconds', '20', '--max-raw-gib', '.08' if full_input else '.01',
               '--profile', 'sensor_probe', '--clock-mode', 'simulation' if simulation_clock else 'system']
    prototype = fixture_messages(0, full_input)
    for topic in prototype:
        command.extend(['--sensor-topic', topic])
    # No physical endpoints or global DDS configuration is changed.
    domain = 191 if full_input else 190 if regress_clock else 189 if publish_clock else 188 if simulation_clock else 187
    os.environ.update(ROS_DOMAIN_ID=str(domain),
                      ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    for key in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER'):
        os.environ.pop(key, None)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sensor_probe_fixture')
    qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = {t: node.create_publisher(type(m), t, qos) for t, m in prototype.items()}
    clock_publisher = node.create_publisher(Clock, '/clock', 10) if publish_clock else None
    child = None
    sent = {t: Counter() for t in publishers}
    try:
        # Do not run against somebody else's nodes, even on this localhost test domain.
        until = time.monotonic()+.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.03)
        if set(node.get_node_names())-{'sensor_probe_fixture'}:
            raise RuntimeError('test ROS domain already occupied')
        with (root/'recorder.log').open('x') as log:
            child = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            until = time.monotonic()+12
            subscriptions = None
            while subscriptions is None:
                if child.poll() is not None or time.monotonic() >= until:
                    raise RuntimeError('recorder startup/discovery failed')
                rclpy.spin_once(node, timeout_sec=.05)
                if ((recording/'ready.json').exists()
                        and all(p.get_subscription_count() for p in publishers.values())
                        and (clock_publisher is None or clock_publisher.get_subscription_count())
                        and 'sensor_bag_recorder' in node.get_node_names()):
                    try:
                        subscriptions = dict(node.get_subscriber_names_and_types_by_node('sensor_bag_recorder', '/'))
                    except Exception as error:
                        # DDS endpoint matching and the node-name graph may arrive separately.
                        if type(error).__name__ != 'NodeNameNonExistentError':
                            raise
            if clock_publisher is not None:
                initial = Clock()
                initial.clock.sec = 10
                for _ in range(5):
                    clock_publisher.publish(initial)
                    rclpy.spin_once(node, timeout_sec=.02)
            for i in range(20 if full_input else 80):
                if clock_publisher is not None:
                    clock = Clock()
                    clock.clock.sec, clock.clock.nanosec = 10, i*10_000_000
                    clock_publisher.publish(clock)
                # Deliberately unknown/zero/regressing header time: preserve, do not repair.
                messages = fixture_messages(i, full_input)
                for topic, message in messages.items():
                    payload = bytes(serialize_message(message))
                    sent[topic][hashlib.sha256(payload).hexdigest()] += 1
                    publishers[topic].publish(message)
                rclpy.spin_once(node, timeout_sec=.05 if full_input else .01)
            if regress_clock:
                backwards = Clock()
                backwards.clock.sec = 2
                for _ in range(5):
                    clock_publisher.publish(backwards)
                    rclpy.spin_once(node, timeout_sec=.02)
                # A sensor callback must observe the backwards recorder clock.
                publishers['/imu/data'].publish(messages['/imu/data'])
            until = time.monotonic()+.25
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.03)
            # Regression is an autonomous stop test. Sending SIGINT here races
            # with rclpy shutdown/restored signal handlers and masks its exit status.
            if not regress_clock and child.poll() is None:
                child.send_signal(signal.SIGINT)
            code = child.wait(timeout=20)
        manifest = json.loads((recording/'manifest.json').read_text())
        try:
            load_manifest(recording)
            replay_rejected = False
        except ValueError:
            replay_rejected = True
        checks = dict(no_command_subscriptions=not ({'/drive', '/drive/safe'} & set(subscriptions)),
                      no_controller_parameters=json.loads((recording/'parameters.json').read_text()) == {},
                      controller_replay_rejected=replay_rejected,
                      explicit_clock_mode=manifest['recorder_use_sim_time'] is simulation_clock,
                      sensor_clock_not_approved=manifest['sensor_acquisition_time_verified'] is False,
                      process_closed=child.poll() is not None)
        details = {}
        if regress_clock:
            checks.update(recorder_clock_regression_rejected=not manifest['complete'] and code == 1,
                          regression_observed=manifest['clock_regressions'] > 0,
                          earlier_samples_preserved=all(s['count'] > 0 for s in manifest['stats'].values()))
        elif simulation_clock and not publish_clock:
            checks.update(no_clock_input_rejected=not manifest['complete'] and code == 1,
                          preclock_observed=manifest['preclock_skipped'] > 0,
                          no_samples_fabricated=all(s['count'] == 0 for s in manifest['stats'].values()))
        else:
            details = audit(recording)
            got = {t: Counter() for t in publishers}
            reader = reader_for(recording, topics=manifest['topics'])
            while reader.has_next():
                topic, data, _ = reader.read_next()
                got[topic][hashlib.sha256(data).hexdigest()] += 1
            checks.update(complete_with_selected_clock=manifest['complete'] and code == 0,
                          exact_original_cdr=all(not (got[t]-sent[t]) and sum(got[t].values()) >= 20 for t in got),
                          original_zero_stamps_retained=all(v > 0 for v in details['source_stamp_zero'].values()),
                          original_regression_retained=all(v > 0 for v in details['source_stamp_regressions'].values()),
                          record_integrity_verified=details['record_integrity_verified'])
            if full_input:
                checks.update(all_sent_samples_preserved=all(got[t] == sent[t] for t in sent),
                              one_motor_topic_only=MOTOR_TOPIC in manifest['topics'] and '/wheel_states' not in manifest['topics'])
        result = dict(case=case, passed=all(checks.values()), checks=checks, details=details,
                      source='synthetic ROS messages only', hardware_verified=False, ready_to_drive=False,
                      command=command, exit_code=code, recorder_subscriptions=subscriptions,
                      sent_counts={t: sum(v.values()) for t, v in sent.items()})
        (root/'report.json').write_text(json.dumps(result, indent=2)+'\n')
        return result
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cases', nargs='+', choices=('system', 'sim_no_clock', 'sim_valid', 'sim_regression', 'full_input'),
                        default=('system', 'sim_no_clock'))
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts'):
        raise ValueError('artifacts output required')
    output.mkdir(parents=True, exist_ok=False)
    sources = [ROOT/p for p in ('scripts/validate_sensor_probe.py', 'scripts/record_sensor_bag.py',
        'scripts/sensor_recording_profile.py', 'scripts/audit_sensor_probe.py', 'scripts/audit_sensor_bag.py',
        'scripts/rosbag_writer_compat.py',
        'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
        'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py')]
    before = {p.relative_to(ROOT).as_posix(): sha256_file(p) for p in sources}
    for p in sources:
        target = output/'sources'/p.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, target)
    if len(set(args.cases)) != len(args.cases):
        raise ValueError('duplicate cases')
    configs = dict(system=(False, False, False), sim_no_clock=(True, False, False),
                   sim_valid=(True, True, False), sim_regression=(True, True, True),
                   full_input=(False, False, False, True))
    results = [trial(output/name, name, *configs[name]) for name in args.cases]
    after = {p.relative_to(ROOT).as_posix(): sha256_file(p) for p in sources}
    result = dict(passed=all(r['passed'] for r in results) and before == after,
                  source_sha256=before, source_unchanged=before == after,
                  cases=[{k: r[k] for k in ('case', 'passed', 'checks')} for r in results],
                  hardware_verified=False, ready_to_drive=False)
    (output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
