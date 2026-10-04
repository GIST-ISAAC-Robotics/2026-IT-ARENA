#!/usr/bin/env python3
"""독립 ROS 보호층에 합성 C1/IMU/모터 피드백을 발행한다. 구동기/실물은 연결하지 않는다."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts'):
        raise ValueError('output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(170 + os.getpid() % 25),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('sensor_input_validator')
    publishers = {
        'scan': node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data),
        'imu': node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data),
        'motor': node.create_publisher(JointState, '/drive_motor/encoder', 100),
        'command': node.create_publisher(AckermannDriveStamped, '/drive', 10),
    }
    rows, cases = [], []
    safe = []
    node.create_subscription(String, '/safety/status', lambda m: rows.append({
        'received_wall_s': time.monotonic(), **json.loads(m.data)}), 100)
    node.create_subscription(AckermannDriveStamped, '/drive/safe',
                             lambda m: safe.append((time.monotonic(), m.drive.speed)), 100)
    sources = [Path(__file__).relative_to(ROOT).as_posix()] + [
        'src/arena_autonomy/arena_autonomy/' + name + '.py'
        for name in ('lidar_safety', 'lidar_observation', 'lidar_motion', 'lidar_motion_ros', 'local_path')]
    report = dict(passed=False, scope='isolated ROS synthetic inputs and independent safety output; no vehicle or MCU',
                  cases=cases, source_sha256={s: hashlib.sha256((ROOT / s).read_bytes()).hexdigest() for s in sources})
    with zipfile.ZipFile(output / 'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT / name, name)
    log = (output / 'lidar_safety.log').open('w')
    command = [sys.executable, '-c', 'from arena_autonomy.lidar_safety import main; main()',
               '--ros-args', '-p', 'lidar_compensation:=both', '-p', 'motion_scan_timing_verified:=true',
               '-p', 'motion_imu_topic:=/imu/data', '-p', 'motion_feedback_mode:=drive_motor_shaft',
               '-p', 'motion_gear_ratio:=15.0', '-p', 'motion_wheel_radius_m:=0.0325']
    child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=log, start_new_session=True)
    last_scan = -math.inf

    def stamp(message, seconds):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
        return message

    def exercise(name, duration=1., value=2., indices=(), gyro='healthy', steering=0., scan_delay=0.,
                 expected_speed=True, expected_reasons=('clear',)):
        nonlocal last_scan
        began = time.monotonic()
        begin_rows, begin_safe = len(rows), len(safe)
        frozen_stamp = node.get_clock().now().nanoseconds * 1e-9
        first_fault = None
        while time.monotonic() - began < duration:
            if child.poll() is not None:
                raise RuntimeError(f'safety child ended: {child.returncode}')
            now = node.get_clock().now().nanoseconds * 1e-9
            # 200 Hz 요청. 실제 호스트 스케줄 지터가 있으면 시간 계약을 완화하지 않는다.
            motor = stamp(JointState(name=['drive_motor_shaft'], position=[0.], velocity=[0.]), now)
            publishers['motor'].publish(motor)
            if gyro != 'drop':
                msg = stamp(Imu(), frozen_stamp if gyro == 'same_stamp' else now)
                msg.angular_velocity.z = 100. if gyro == 'range' else 0.
                publishers['imu'].publish(msg)
            drive = stamp(AckermannDriveStamped(), now)
            drive.drive.speed, drive.drive.steering_angle = 1., steering
            publishers['command'].publish(drive)
            if now - last_scan >= .1:
                scan = stamp(LaserScan(angle_min=-math.pi, angle_max=math.pi-2*math.pi/500,
                    angle_increment=2*math.pi/500, range_min=.05, range_max=12.,
                    time_increment=.1/500, scan_time=.1, ranges=[2.]*500), now-.1-scan_delay)
                scan.header.frame_id = 'laser_frame'
                for index in indices:
                    scan.ranges[index] = value
                publishers['scan'].publish(scan)
                last_scan = now
                if first_fault is None:
                    first_fault = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.002)
            rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(.003)
        tail_start = began + duration * .6
        sample = [r for r in rows[begin_rows:] if r['received_wall_s'] >= tail_start]
        received_safe = [v for t, v in safe[begin_safe:] if t >= tail_start]
        reason_counts = {r: sum(x['reason'] == r for x in sample) for r in sorted({x['reason'] for x in sample})}
        passed = (len(sample) >= 5 and len(received_safe) >= 5 and
                  all(r['reason'] in expected_reasons for r in sample) and
                  all((r['safe_speed_mps'] > .5 if expected_speed else r['safe_speed_mps'] == 0.) for r in sample) and
                  all((v > .5 if expected_speed else v == 0.) for v in received_safe))
        stopped = next((r for r in rows[begin_rows:] if not expected_speed and
                        r['received_wall_s'] >= (first_fault or began) and r['safe_speed_mps'] == 0.), None)
        cases.append(dict(name=name, passed=passed, expected_positive=expected_speed,
                          status_samples=len(sample), safe_command_samples=len(received_safe), reasons=reason_counts,
                          first_stop_after_first_scan_publish_wall_ms=(
                              1000*(stopped['received_wall_s'] - first_fault) if stopped and first_fault else None)))

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and (publishers['scan'].get_subscription_count() == 0 or
                                              node.count_publishers('/drive/safe') == 0):
            if child.poll() is not None:
                raise RuntimeError('safety child failed during startup')
            rclpy.spin_once(node, timeout_sec=.05)
        if publishers['scan'].get_subscription_count() != 1:
            raise RuntimeError('expected exactly one isolated safety subscriber')
        exercise('healthy_initial', duration=1.5)
        for steering in (0., .25, -.25):
            exercise(f'positive_inf_single_steer_{steering}', value=math.inf, indices=[250], steering=steering,
                     expected_speed=False, expected_reasons=('front_unobserved',))
            exercise(f'finite_recovery_steer_{steering}', steering=steering)
        for name, value, indices in [('positive_inf_sector', math.inf, range(223, 278)),
                                     ('all_positive_inf', math.inf, range(500)),
                                     ('nan_front', math.nan, [250]), ('too_close', .01, [250]),
                                     ('above_range', 13., [250])]:
            exercise(name, value=value, indices=indices, expected_speed=False, expected_reasons=('front_unobserved',))
        exercise('finite_recovery')
        exercise('scan_delay_during_run', scan_delay=.3, expected_speed=False, expected_reasons=('stale_input',))
        exercise('scan_delay_recovery')
        for kind in ('range', 'same_stamp', 'drop'):
            exercise('gyro_' + kind, gyro=kind, expected_speed=False,
                     expected_reasons=('motion_stale', 'motion_gap'))
            exercise('gyro_' + kind + '_recovery')
        exercise('fresh_constant_zero_is_not_stuck')
        report['passed'] = all(c['passed'] for c in cases)
    except Exception as error:
        report['error'] = repr(error)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)
                report['forced_cleanup'] = True
        report['child_exit_code'] = child.returncode
        report['passed'] &= child.returncode == 0 and not report.get('forced_cleanup', False)
        report['subscriptions'] = {kind: topic for kind, topic in (
            ('scan', '/scan'), ('imu', '/imu/data'), ('motor', '/drive_motor/encoder'), ('command', '/drive'))}
        # 원시 ROS 표본 대신 작고 해석 가능한 상태 전환 증거를 보존한다.
        transitions = []
        for row in rows:
            if not transitions or row['reason'] != transitions[-1]['reason']:
                transitions.append(row)
        report['status_transitions'] = transitions
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'passed': report['passed'], 'cases': len(cases), 'output': str(output)}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
