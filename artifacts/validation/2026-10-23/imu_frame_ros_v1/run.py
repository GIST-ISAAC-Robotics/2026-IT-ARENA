"""Installed MotionInput over real local DDS. Synthetic clock/sensors, no drive path."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
import arena_autonomy.lidar_motion_ros as motion
from arena_autonomy.lidar_motion import MotionUnavailable
import arena_vehicle_interface.drive_feedback as feedback

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREFIX = ROOT/'build/install_audit_20261023_install'
checks, errors, received = [], [], {'strict': [], 'compatible': []}
source_hashes = {}
for module, package in ((motion, 'arena_autonomy'), (feedback, 'arena_vehicle_interface')):
    file = Path(module.__file__).resolve()
    assert file.is_relative_to(PREFIX)
    src = ROOT/'src'/package/package/file.name
    actual = hashlib.sha256(file.read_bytes()).hexdigest()
    assert actual == hashlib.sha256(src.read_bytes()).hexdigest()
    source_hashes[str(file)] = actual

def check(name, condition):
    checks.append(dict(name=name, passed=bool(condition)))
    if not condition:
        raise AssertionError(name)

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
executor = SingleThreadedExecutor()
nodes = []
adapters = {}
try:
    fixture = Node('imu_frame_fixture')
    nodes.append(fixture)
    executor.add_node(fixture)
    for name, frame in (('strict','imu_link'), ('compatible','')):
        options = {'use_sim_time': True, 'motion_imu_topic': '/imu/data',
                   'motion_feedback_mode': 'drive_motor_shaft', 'motion_gear_ratio': 15.,
                   'motion_wheel_radius_m': .0325, 'motion_imu_roll_rad': math.pi/2}
        if frame:
            options['motion_imu_frame_id'] = frame
        node = Node('imu_frame_'+name, parameter_overrides=[Parameter(k,value=v) for k,v in options.items()])
        nodes.append(node)
        executor.add_node(node)
        adapters[name] = motion.MotionInput(node, record=received[name].append)
    clock_pub = fixture.create_publisher(Clock, '/clock', 10)
    imu_pub = fixture.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    motor_pub = fixture.create_publisher(JointState, '/drive_motor/encoder', 10)
    def until(predicate, timeout=5.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError('fixture convergence/delivery timeout')
            executor.spin_once(timeout_sec=.01)
    until(lambda: imu_pub.get_subscription_count()==2 and motor_pub.get_subscription_count()==2
          and clock_pub.get_subscription_count()==2)
    check('only_owned_nodes', set(fixture.get_node_names())=={n.get_name() for n in nodes})
    check('single_motor_configuration', all(a.feedback_topic=='/drive_motor/encoder'
          and a.gear_ratio==15. and a.history.config.wheel_radius_m==.0325 for a in adapters.values()))
    check('frame_default_unchanged', adapters['compatible'].expected_imu_frame=='' and adapters['strict'].expected_imu_frame=='imu_link')
    stamp_ns = 0
    def advance(ns):
        global stamp_ns
        stamp_ns = ns
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(ns, 1_000_000_000)
        clock_pub.publish(clock)
        until(lambda: all(n.get_clock().now().nanoseconds==ns for n in nodes[1:]))
    def stamp(message):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns,1_000_000_000)
    def send_imu(frame='imu_link', value=.2):
        before = {k: len(rows) for k,rows in received.items()}
        msg=Imu()
        stamp(msg)
        msg.header.frame_id=frame
        msg.angular_velocity.y=value
        imu_pub.publish(msg)
        until(lambda: all(len(received[k])==n+1 for k,n in before.items()))
    def send_motor(valid=True):
        before={k:a.counts['drive_motor']+a.counts['rejected'] for k,a in adapters.items()}
        msg=JointState()
        stamp(msg)
        msg.name=['drive_motor_shaft'] if valid else ['rear_left_wheel_joint','rear_right_wheel_joint']
        msg.position=[1.] if valid else [1.,1.]
        msg.velocity=[150.] if valid else [10.,10.]
        motor_pub.publish(msg)
        until(lambda: all(a.counts['drive_motor']+a.counts['rejected']==before[k]+1 for k,a in adapters.items()))
    for i in range(3):
        advance(100_000_000_000+i*10_000_000)
        send_motor()
        send_imu()
    strict, compatible = adapters['strict'], adapters['compatible']
    check('valid_imu_and_mounting_projection', strict.counts['gyro']==3 and all(abs(v-.2)<1e-12 for v in strict.history.gyro.values))
    check('motor_ratio_and_radius', strict.counts['drive_motor']==3 and all(abs(v-.325)<1e-12 for v in strict.history.wheels.values))
    good_times=list(strict.history.gyro.times)
    for i, frame in enumerate(('', 'wrong', '/imu_link', 'base_link', 'IMU_link'), start=3):
        advance(100_000_000_000+i*10_000_000)
        send_motor()
        send_imu(frame)
        check('reject_frame_'+str(i), strict.history.gyro.times==good_times and received['strict'][-1]['accepted'] is False)
        check('legacy_accept_frame_'+str(i), received['compatible'][-1]['accepted'] is True)
    check('no_zero_substitution', strict.history.gyro.values==[.2,.2,.2])
    try:
        strict.history.poses([100.06,100.07],100.07)
        rejection=None
    except MotionUnavailable as error:
        rejection=str(error)
    check('rejected_stream_expires', rejection=='motion_stale')
    compatible.history.poses([100.06,100.07],100.07)
    check('compatible_stream_still_available', True)
    send_imu('imu_link')  # Same timestamp as rejected frame must remain available for a valid sample.
    check('same_rejected_timestamp_not_consumed', strict.history.gyro.times[-1]==100.07)
    advance(100_080_000_000)
    send_motor()
    send_imu()
    strict.history.poses([100.07,100.08],100.08)
    check('fresh_window_recovers', True)
    previous=list(strict.history.gyro.times)
    advance(100_090_000_000)
    send_imu(value=float('nan'))
    check('correct_frame_still_rejects_nonfinite', strict.history.gyro.times==previous and strict.last_gyro_rejection=='gyro_nonfinite')
    send_imu(value=strict.gyro_abs_limit)
    check('correct_frame_still_rejects_saturation', strict.history.gyro.times==previous and strict.last_gyro_rejection=='gyro_range_or_saturation')
    wheels=list(strict.history.wheels.times)
    send_motor(False)
    check('legacy_wheel_names_not_motor_feedback', strict.history.wheels.times==wheels)
    advance(90_000_000_000)
    send_imu()
    check('clock_rollback_discards_prior_history', strict.counts['resets']==1 and strict.history.gyro.times==[90.] and not strict.history.wheels.times)
    check('no_drive_topics', not any(topic in ('/drive','/drive/safe') or topic.startswith('/actuation/')
                                    for topic,_ in fixture.get_topic_names_and_types()))
except Exception as error:
    errors.append(repr(error))
finally:
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in source_hashes.items())
report=dict(passed=not errors and all(c['passed'] for c in checks) and unchanged,
            checks=checks, errors=errors, source_sha256=source_hashes, installed_sources_unchanged=unchanged,
            input_events=received, synthetic_clock_and_sensors=True, ros_node_cleanup_complete=True,
            actual_imu_alignment_verified=False, hardware_verified=False, gazebo_executed=False,
            drive_commands_published=False, controller_stop_path_verified=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report,stream,indent=2,allow_nan=False)
print(json.dumps(dict(passed=report['passed'],checks=len(checks),errors=errors)))
raise SystemExit(0 if report['passed'] else 1)
