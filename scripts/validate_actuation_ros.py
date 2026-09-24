#!/usr/bin/env python3
"""ROS 독립 프로세스 구동 계약 검사. 합성 엔코더/종방향 모형이며 Gazebo가 아니다."""
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

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / 'artifacts'):
        raise ValueError('output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(140 + os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='')
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('actuation_ros_validator', parameter_overrides=[Parameter('use_sim_time', value=True)])
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    wheel_pub = node.create_publisher(JointState, '/wheel_states', 10)
    drive_pub = node.create_publisher(AckermannDriveStamped, '/test/safe_command', 1)
    state, events, cases, children, logs = {}, [], [], [], []
    report = {'passed': False, 'scope': 'ROS processes with synthetic encoder and simple plant, not Gazebo or hardware',
              'cases': cases, 'source_sha256': {}}
    for relative in ['scripts/validate_actuation_ros.py', *[
            'src/arena_vehicle_interface/arena_vehicle_interface/' + name + '.py'
            for name in ('actuation_ros', 'applied_guard', 'actuation_contract', 'actuation_wire')]]:
        report['source_sha256'][relative] = hashlib.sha256((REPO / relative).read_bytes()).hexdigest()
    t, velocity, position = 1., 0., 0.
    send_drive = send_encoder = True
    freeze = False
    target = .6
    last_wheel = last_drive = -1.
    sequence = 0

    def remember(key, value):
        state[key] = value
        events.append({'time_sim_s': t, 'wall_s': time.monotonic(), 'topic': key, 'value': value})

    node.create_subscription(String, '/actuation/status', lambda m: remember('mcu', json.loads(m.data)), 100)
    node.create_subscription(String, '/actuation/output_status', lambda m: remember('sink', json.loads(m.data)), 100)
    node.create_subscription(Twist, '/sim/cmd_vel', lambda m: state.update(applied=m.linear.x), 10)
    clients = {kind: node.create_client(Trigger, '/actuation/' + kind) for kind in ('arm', 'clear_stop', 'stop')}

    def start(kind):
        log = (output / f'{kind}_{len(children)}.log').open('w')
        logs.append(log)
        command = [sys.executable, '-c', f'from arena_vehicle_interface.actuation_ros import {kind}_main; {kind}_main()',
                   '--ros-args', '-p', 'use_sim_time:=true', '-p', 'wall_timeout_s:=0.5']
        if kind == 'host':
            command += ['-r', '/drive/safe:=/test/safe_command']
        child = subprocess.Popen(command, cwd=REPO, stdout=log, stderr=log, start_new_session=True)
        children.append(child)
        return child

    def stop(child):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)
                report.setdefault('forced_pids', []).append(child.pid)

    def pump(wall_seconds):
        nonlocal t, velocity, position, last_wheel, last_drive, sequence
        until = time.monotonic() + wall_seconds
        while time.monotonic() < until:
            if not freeze:
                t += .002
                velocity += max(-.004, min(.004, state.get('applied', 0.) - velocity))
                position += velocity * .002
            stamp = Clock()
            stamp.clock.sec = int(t)
            stamp.clock.nanosec = round((t % 1) * 1e9)
            if stamp.clock.nanosec >= 1_000_000_000:
                stamp.clock.sec += 1
                stamp.clock.nanosec = 0
            clock_pub.publish(stamp)
            if send_encoder and t - last_wheel >= .0099:
                counts = round(position / .025 / math.tau * 2048)
                wheel = JointState()
                wheel.header.stamp = stamp.clock
                wheel.name = ['rear_left_wheel_joint', 'rear_right_wheel_joint']
                wheel.position = [counts * math.tau / 2048] * 2
                wheel_pub.publish(wheel)
                last_wheel = t
            if send_drive and t - last_drive >= .0199:
                drive = AckermannDriveStamped()
                drive.header.stamp = stamp.clock
                drive.drive.speed = target
                drive_pub.publish(drive)
                last_drive = t
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
            time.sleep(.004)

    def wait_for(predicate, wall=8.):
        end = time.monotonic() + wall
        while time.monotonic() < end:
            pump(.02)
            if predicate():
                return
        raise AssertionError('condition timeout: ' + json.dumps(state))

    def call(kind, expected=True):
        wait_for(lambda: clients[kind].service_is_ready())
        if kind == 'arm' and expected:
            wait_for(lambda: state.get('mcu', {}).get('output_ready', False))
        future = clients[kind].call_async(Trigger.Request())
        wait_for(future.done)
        response = future.result()
        if response.success != expected:
            raise AssertionError(f'{kind}: {response.success} {response.message}')
        return response.message

    def record(name, condition):
        passed = bool(condition)
        cases.append({'name': name, 'passed': passed, 'sim_s': t, 'model_speed_mps': velocity,
                      'mcu': state.get('mcu'), 'sink': state.get('sink')})
        print(f'{name}: {passed}', flush=True)
        if not passed:
            raise AssertionError(name)

    def stopped():
        return abs(velocity) < .01 and state.get('applied') == 0.

    def stationary():
        wait_for(stopped)
        pump(.9)

    def rearm():
        stationary()
        call('clear_stop')
        pump(.1)
        call('arm')
        wait_for(lambda: velocity > .5)

    try:
        host, mcu, sink = start('host'), start('mcu'), start('actuator')
        wait_for(lambda: state.get('mcu', {}).get('feedback_valid', False))
        pump(1.)
        record('boot_no_auto_arm', stopped() and state['mcu']['state'] == 'DISARMED')
        call('arm')
        wait_for(lambda: velocity > .5)
        record('explicit_arm_drive', state['mcu']['state'] == 'ACTIVE')
        send_drive = False
        stationary()
        record('source_loss_latched_stop', state['mcu']['reason'] == 'command_expired')
        send_drive = True
        pump(.5)
        record('source_restore_no_restart', stopped())
        call('clear_stop')
        pump(.6)
        record('clear_only_no_restart', stopped() and state['mcu']['state'] == 'DISARMED')
        call('arm')
        wait_for(lambda: velocity > .5)
        call('stop')
        stationary()
        record('software_stop_latched', state['mcu']['state'] == 'ESTOP_LATCHED')
        rearm()
        stop(host)
        stationary()
        host = start('host')
        pump(1.5)
        record('host_restart_no_restart', stopped() and state['mcu']['state'] == 'FAULT_LATCHED')
        rearm()
        old_boot = state['mcu']['boot']
        stop(mcu)
        stationary()
        record('mcu_process_loss_output_stop', state['sink']['blocked'])
        mcu = start('mcu')
        wait_for(lambda: state['mcu']['boot'] != old_boot and state['mcu']['feedback_valid'])
        pump(.9)
        record('mcu_restart_no_restart', stopped() and state['mcu']['state'] == 'DISARMED')
        call('arm')
        wait_for(lambda: velocity > .5)
        send_encoder = False
        stationary()
        record('encoder_loss_stop', state['mcu']['reason'] == 'encoder_timeout')
        send_encoder = True
        rearm()
        freeze = True
        pump(.8)
        record('frozen_clock_zero_output', state['applied'] == 0. and state['mcu']['state'] == 'FAULT_LATCHED')
        freeze = False
        stationary()
        call('clear_stop', expected=False)
        record('clock_fault_requires_process_restart', stopped())
        stop(mcu)
        old_boot = state['mcu']['boot']
        mcu = start('mcu')
        wait_for(lambda: state['mcu']['boot'] != old_boot and state['mcu']['feedback_valid'])
        pump(.9)
        call('arm')
        wait_for(lambda: velocity > .5)
        record('clock_fault_explicit_restart_recovery', velocity > .5)
        t -= .5
        last_wheel = last_drive = t - .1
        pump(.1)
        wait_for(lambda: state['mcu']['state'] == 'FAULT_LATCHED')
        stationary()
        record('clock_regression_zero_no_restart', stopped())
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
    finally:
        for child in reversed(children):
            stop(child)
        report['process_exit_codes'] = [c.returncode for c in children]
        report['remaining_pids'] = [c.pid for c in children if c.poll() is None]
        report['passed'] &= not report.get('forced_pids') and not report['remaining_pids'] and all(c.returncode == 0 for c in children)
        for log in logs:
            log.close()
        (output / 'events.json').write_text(json.dumps(events, indent=2), encoding='utf-8')
        (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps({k: v for k, v in report.items() if k != 'cases'}, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
