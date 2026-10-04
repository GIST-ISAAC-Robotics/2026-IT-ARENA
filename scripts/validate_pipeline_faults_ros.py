#!/usr/bin/env python3
"""실제 ROS 파이프라인의 RGB/프로세스 중단 검사 (가상 직선 센서·종방향 모형).

Linux ROS overlay에서 실행한다. 센서와 /clock만 발행하며 명령 보호층을 우회하지
않는다. 수신 계측은 ROS/DDS와 검증기 스케줄을 포함하며 제동/지연 보증이 아니다.
"""
import argparse
import ast
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
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
DT = .005
MODULES = {
    'arena_autonomy': ('local_pursuit', 'wall_follow', 'vision_process', 'core', 'local_path',
                       'lidar_safety', 'lidar_motion', 'lidar_motion_ros', 'lidar_observation'),
    'arena_vehicle_interface': ('actuation_ros', 'actuation_contract', 'actuation_wire',
                                'applied_guard', 'drive_feedback', 'node_lifecycle',
                                'queued_log', 'timing_probe'),
}
SOURCES = [f'src/{package}/{package}/{name}.py'
           for package, names in MODULES.items() for name in names]
PROVENANCE = ['tests/test_pipeline_fault_validation.py', 'src/arena_description/config/vehicle.yaml',
              'config/tracks/official_v2026.09.14.yaml']
ZERO_TOPICS = ('applied', 'sink', 'mcu')
ZERO_GAP_SIM_S, ZERO_GAP_WALL_S = .25, 3.
ROLES = {
    'controller': ('arena_autonomy.local_pursuit', 'main', 'local_pursuit'),
    'safety': ('arena_autonomy.lidar_safety', 'main', 'lidar_safety'),
    'host': ('arena_vehicle_interface.actuation_ros', 'host_main', 'actuation_host'),
    'mcu': ('arena_vehicle_interface.actuation_ros', 'mcu_main', 'virtual_mcu'),
    'sink': ('arena_vehicle_interface.actuation_ros', 'actuator_main', 'guarded_sim_actuator'),
}


def stamp_parts(seconds):
    return divmod(round(seconds * 1e9), 1_000_000_000)


def signal_rgb(color):
    """테스트 모듈에 의존하지 않는 실제 픽셀 신호등 (rgb8)."""
    import cv2
    import numpy as np
    rgb = np.full((480, 848, 3), 100, np.uint8)
    cv2.rectangle(rgb, (250, 130), (390, 180), (3, 3, 3), -1)
    for name, x, value in [('red', 280, (255, 4, 1)), ('yellow', 320, (255, 180, 1)),
                           ('green', 360, (1, 255, 10))]:
        cv2.circle(rgb, (x, 155), 10, value if color == name else (0, 0, 0), -1)
    return rgb.tobytes()


def corridor_range(angle, origin_x=0., origin_y=0., half_length=6., half_width=.425):
    """유한 직사각 벽에 실제로 교차한 가장 가까운 ray 거리."""
    if not (-half_length < origin_x < half_length and -half_width < origin_y < half_width):
        raise ValueError('ray origin must be inside the fixture')
    dx, dy = math.cos(angle), math.sin(angle)
    distances = []
    if abs(dx) > 1e-12:
        distances.append(((half_length if dx > 0 else -half_length) - origin_x) / dx)
    if abs(dy) > 1e-12:
        distances.append(((half_width if dy > 0 else -half_width) - origin_y) / dy)
    return min(distances)


@dataclass
class ToyPlant:
    """실물 제동/PI가 아닌 임시 ±2 m/s² 종방향 모델."""
    speed: float = 0.
    distance: float = 0.
    acceleration: float = 2.

    def step(self, applied, dt=DT):
        if not math.isfinite(applied) or applied < 0:
            raise ValueError('invalid applied speed')
        before = self.speed
        self.speed += max(-self.acceleration * dt, min(self.acceleration * dt, applied - self.speed))
        self.distance += (before + self.speed) * .5 * dt


def proc_identity(pid):
    """/proc starttime로 PID 재사용을 거절한다. comm의 공백/괄호도 허용."""
    raw = (Path('/proc') / str(pid) / 'stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    return dict(pid=pid, state=fields[0], parent=int(fields[1]), group=int(fields[2]),
                session=int(fields[3]), start_ticks=int(fields[19]))


def same_process(saved, current):
    return all(saved[key] == current[key] for key in ('pid', 'start_ticks', 'group', 'session'))


def zero_output(key, value):
    return (value if key == 'applied' else
            value['speed_mps' if key == 'sink' else 'target_speed_mps']) == 0.


def vision_cleanup_results(lines, intentional_kill=False):
    """프로덕션 경고의 literal만 해석한다. 임의 코드를 실행하지 않는다."""
    results = []
    for line in lines:
        if 'Vision process cleanup: ' not in line:
            continue
        value = ast.literal_eval(line.split('Vision process cleanup: ', 1)[1])
        if (type(value) is not dict or type(value.get('forced')) is not bool
                or type(value.get('exitcode')) is not int):
            raise ValueError('invalid vision cleanup warning')
        value['accepted'] = not value['forced'] and (
            value['exitcode'] == 0 or intentional_kill and value['exitcode'] == -signal.SIGKILL)
        results.append(value)
    return results


class Monitor:
    """전체 원시 표본 대신 bounded history와 상태 전환을 보존한다."""
    def __init__(self):
        self.sequence = 0
        self.latest = {}
        self.rows = deque(maxlen=100_000)
        self.events = []
        self.signatures = {}
        self.zero_watch = None

    def begin_zero_watch(self, case):
        self.zero_watch = dict(case=case, processed=case['sequence'],
                               latest={key: self.latest[key] for key in ZERO_TOPICS}, counts={key: 0 for key in ZERO_TOPICS},
                               maximum_sim_gap_s=0., maximum_wall_gap_s=0.)
        case['zero_receipt_bounds'] = dict(sim_gap_s=ZERO_GAP_SIM_S, wall_gap_s=ZERO_GAP_WALL_S,
            interpretation='provisional sampled-monitor bounds, not delivery or physical safety guarantee')

    def check_zero_watch(self, now_wall, now_sim):
        watch = self.zero_watch
        if watch is None:
            return
        if self.rows and watch['processed'] < self.rows[0]['sequence'] - 1:
            raise AssertionError('zero observation history lost')
        pending = []
        for row in reversed(self.rows):
            if row['sequence'] <= watch['processed']:
                break
            pending.append(row)
        for row in reversed(pending):
            key = row['topic']
            if key in ZERO_TOPICS:
                if not zero_output(key, row['value']):
                    watch['case']['nonzero_excursion'] = row
                    raise AssertionError('no-resume nonzero excursion: ' + key)
                before = watch['latest'][key]
                sim_gap = row['received_sim_s'] - before['received_sim_s']
                wall_gap = row['received_wall_s'] - before['received_wall_s']
                watch['maximum_sim_gap_s'] = max(watch['maximum_sim_gap_s'], sim_gap)
                watch['maximum_wall_gap_s'] = max(watch['maximum_wall_gap_s'], wall_gap)
                if sim_gap > ZERO_GAP_SIM_S or wall_gap > ZERO_GAP_WALL_S:
                    raise AssertionError('zero receipt gap: ' + key)
                watch['latest'][key] = row
                watch['counts'][key] += 1
            watch['processed'] = row['sequence']
        for key, row in watch['latest'].items():
            if (now_wall - row['received_wall_s'] > ZERO_GAP_WALL_S
                    or now_sim - row['received_sim_s'] > ZERO_GAP_SIM_S):
                raise AssertionError('zero receipt missing: ' + key)
        watch['case']['zero_observation'] = dict(counts=dict(watch['counts']),
            maximum_sim_gap_s=watch['maximum_sim_gap_s'], maximum_wall_gap_s=watch['maximum_wall_gap_s'])

    def end_zero_watch(self):
        if any(count == 0 for count in self.zero_watch['counts'].values()):
            raise AssertionError('zero hold did not observe every output topic')
        self.zero_watch = None

    def receive(self, key, value, sim_s, source_s=None):
        self.sequence += 1
        row = dict(sequence=self.sequence, received_wall_s=time.monotonic(),
                   received_sim_s=sim_s, source_sim_s=source_s, topic=key, value=value)
        self.rows.append(row)
        self.latest[key] = row
        if isinstance(value, dict):
            signature = tuple(value.get(k) for k in ('state', 'reason', 'blocked', 'started'))
            if key == 'autonomy':
                signature += (value.get('vision', {}).get('fault'),)
        else:
            signature = value > 0
        if signature != self.signatures.get(key):
            self.signatures[key] = signature
            self.events.append(row)

    def fresh(self, key, after, predicate, now_wall, now_sim, max_age=.25):
        row = self.latest.get(key)
        return bool(row and row['sequence'] > after and now_wall - row['received_wall_s'] < .5
                    and 0 <= now_sim - row['received_sim_s'] < max_age
                    and (row['source_sim_s'] is None or
                         0 <= now_sim - row['source_sim_s'] < max_age)
                    and predicate(row['value']))


class OwnedProcesses:
    def __init__(self, output, report):
        self.output, self.report = output, report
        self.children = []
        self.current = {}
        self.vision = []
        self.report['processes'] = []
        self.report['signals'] = []
        self.report['session_observations'] = []
        self.session_members = {}

    def observe_session(self, owned, phase):
        leader = owned['record']['identity']
        try:
            current_leader = proc_identity(leader['pid'])
        except FileNotFoundError:
            current_leader = None
        members = []
        if current_leader is None or same_process(leader, current_leader):
            for path in Path('/proc').iterdir():
                if not path.name.isdigit():
                    continue
                try:
                    current = proc_identity(int(path.name))
                except (FileNotFoundError, ProcessLookupError):
                    continue
                if current['session'] == leader['session'] and current['start_ticks'] >= leader['start_ticks']:
                    members.append(current)
                    self.session_members[(current['pid'], current['start_ticks'])] = current
        self.report['session_observations'].append(dict(role=owned['record']['role'],
            leader=leader, phase=phase, wall_s=time.monotonic(), members=members,
            leader_reused=bool(current_leader and not same_process(leader, current_leader))))
        return members

    def start(self, role):
        module, entry, _ = ROLES[role]
        log_path = self.output / f'{role}_{len(self.children)}.log'
        log = log_path.open('w', encoding='utf-8')
        # 각 자식의 실제 import 위치·해시도 수집한다.
        bootstrap = (
            'import hashlib,importlib,json; '
            f'names={repr([f"{p}.{n}" for p, ns in MODULES.items() for n in ns])}; '
            'mods=[importlib.import_module(n) for n in names]; '
            'print("RUNTIME_SOURCE_MANIFEST="+json.dumps({m.__name__:{"path":m.__file__, '
            '"sha256":hashlib.sha256(open(m.__file__,"rb").read()).hexdigest()} for m in mods}),flush=True); '
            f'from {module} import {entry}; {entry}()'
        )
        parameters = dict(use_sim_time=True)
        if role in ('controller', 'safety'):
            parameters.update(lidar_compensation='both', motion_scan_timing_verified=True,
                              motion_imu_topic='/imu/data', motion_feedback_mode='drive_motor_shaft',
                              motion_gear_ratio=15., motion_wheel_radius_m=.0325,
                              motion_lidar_x_m=.06, motion_rear_axle_x_m=-.0675,
                              wheelbase_m=.135, max_steering_angle_rad=.37)
        if role == 'controller':
            parameters.update(vision_execution='process', max_speed_mps=.6,
                              target_wall_distance_m=.425, lidar_x_m=.06,
                              image_timeout_s=1., control_rate_hz=50.)
        if role == 'mcu':
            parameters.update(feedback_mode='drive_motor_shaft', wheel_radius_m=.0325,
                              ticks_per_revolution=28, gear_ratio=15.)
        if role == 'sink':
            parameters['wheelbase_m'] = .135
        command = [sys.executable, '-c', bootstrap, '--ros-args']
        for name, value in parameters.items():
            command.extend(['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}'])
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=log, start_new_session=True)
        identity = proc_identity(child.pid)
        record = dict(role=role, pid=child.pid, identity=identity, log=log_path.name,
                      command=command, parameters=parameters, expected_exit=0)
        owned = dict(child=child, log=log, record=record)
        self.children.append(owned)
        self.current[role] = owned
        self.report['processes'].append(record)
        return child

    def register_vision(self, pid):
        controller = self.current['controller']['record']['identity']
        current = proc_identity(pid)
        if (current['parent'] != controller['pid'] or current['group'] != controller['group']
                or current['session'] != controller['session']):
            raise RuntimeError('vision PID ownership verification failed')
        if not any(v['pid'] == pid and v['start_ticks'] == current['start_ticks'] for v in self.vision):
            self.vision.append(current)
            self.report.setdefault('vision_processes', []).append(dict(identity=current, expected_signal=None,
                exit_measurement='not validator child; parent result/log and verified /proc disappearance only'))
        return current

    def send(self, role, sig, *, vision=None, phase='fault'):
        owned = self.current[role]
        saved = vision or owned['record']['identity']
        current = proc_identity(saved['pid'])
        if not same_process(saved, current):
            raise RuntimeError('owned PID changed before signal')
        if vision and current['parent'] != owned['child'].pid:
            raise RuntimeError('vision parent changed before signal')
        members = self.observe_session(owned, 'before_' + phase + '_' + sig.name)
        self.report['signals'].append(dict(role='vision' if vision else role,
            pid=saved['pid'], signal=sig.name, wall_s=time.monotonic(), phase=phase,
            scope='owned_process_group' if sig == signal.SIGKILL and vision is None else 'single_owned_pid',
            group_members=members if sig == signal.SIGKILL and vision is None else []))
        if vision and phase == 'fault':
            for record in self.report['vision_processes']:
                if same_process(record['identity'], saved) and sig == signal.SIGKILL:
                    record['expected_signal'] = sig.name
                    record['expected_exit'] = -signal.SIGKILL
                    record['fault_controller_identity'] = owned['record']['identity']
        elif role == 'controller' and sig == signal.SIGKILL and phase == 'fault':
            for record in self.report.get('vision_processes', []):
                if record['identity']['parent'] == saved['pid']:
                    record['expected_signal'] = sig.name
                    record['expected_exit'] = -signal.SIGKILL
        if sig == signal.SIGKILL and vision is None:
            owned['record']['expected_exit'] = -signal.SIGKILL
            # Own session only; also kill controller's known vision and resource tracker.
            os.killpg(saved['group'], sig)
        else:
            os.kill(saved['pid'], sig)

    def check(self):
        for owned in self.children:
            code = owned['child'].poll()
            if code is not None and code != owned['record']['expected_exit']:
                raise RuntimeError(f"unexpected {owned['record']['role']} exit {code}")

    def stop(self, role):
        owned = self.current[role]
        child = owned['child']
        if child.poll() is None:
            self.send(role, signal.SIGCONT, phase='cleanup')
            for saved in self.vision:
                try:
                    current = proc_identity(saved['pid'])
                    if same_process(saved, current) and current['parent'] == child.pid:
                        self.send(role, signal.SIGCONT, vision=saved, phase='cleanup')
                except FileNotFoundError:
                    pass
            self.send(role, signal.SIGINT, phase='cleanup')
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                owned['record']['forced_cleanup'] = True
                self.send(role, signal.SIGKILL, phase='cleanup')
                child.wait(timeout=3)
        self.observe_session(owned, 'after_cleanup')

    def cleanup(self):
        errors = []
        for owned in reversed(self.children):
            role = owned['record']['role']
            if self.current.get(role) is owned:
                try:
                    self.stop(role)
                except Exception as error:
                    errors.append(repr(error))
            owned['record']['exit_code'] = owned['child'].poll()
            owned['log'].close()
            try:
                intentional = any(v.get('expected_exit') == -signal.SIGKILL
                    and v.get('fault_controller_identity') == owned['record']['identity']
                    for v in self.report.get('vision_processes', []))
                warnings = vision_cleanup_results((self.output / owned['record']['log']).read_text(
                    errors='replace').splitlines(), intentional)
                owned['record']['vision_cleanup_results'] = warnings
                if any(not result['accepted'] for result in warnings):
                    errors.append('unexpected vision cleanup: ' + owned['record']['log'])
            except Exception as error:
                errors.append('vision cleanup audit: ' + repr(error))
        leftovers = []
        for record in self.report.get('vision_processes', []):
            saved = record['identity']
            try:
                current = proc_identity(saved['pid'])
                if same_process(saved, current) and current['state'] != 'Z':
                    leftovers.append(saved['pid'])
                    record['final_observation'] = 'owned process still alive'
                else:
                    record['final_observation'] = 'reaped, replaced or zombie; no live owned process'
            except FileNotFoundError:
                record['final_observation'] = 'absent from /proc'
        self.report['remaining_vision_pids'] = leftovers
        session_leftovers = []
        for owned in self.children:
            self.observe_session(owned, 'final')
        for saved in self.session_members.values():
            try:
                current = proc_identity(saved['pid'])
                if same_process(saved, current) and current['state'] != 'Z':
                    session_leftovers.append(current)
            except FileNotFoundError:
                pass
        self.report['remaining_session_members'] = session_leftovers
        self.report['cleanup_errors'] = errors
        return not errors and not leftovers and not session_leftovers and all(
            o['record']['exit_code'] == o['record']['expected_exit']
            and not o['record'].get('forced_cleanup') for o in self.children)


class PipelineValidator:
    def __init__(self, output, wall_cap=480., sleep_s=.005, scenario='full', cycles=1):
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from geometry_msgs.msg import Twist
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, Imu, JointState, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
        self.rclpy, self.types = rclpy, dict(clock=Clock, image=Image, imu=Imu,
                                           motor=JointState, scan=LaserScan)
        self.Trigger = Trigger
        self.output, self.sleep_s = output, sleep_s
        self.scenario, self.cycles = scenario, cycles
        self.deadline = time.monotonic() + wall_cap
        self.t, self.plant, self.monitor = 1., ToyPlant(), Monitor()
        self.history = deque([(self.t, 0.)], maxlen=2000)
        self.spec = MotorEncoderSpec(28, 15., .0325)
        self.report = dict(passed=False, cases=[], scope='synthetic straight RGB/LiDAR/IMU/motor sensors, '
                           'real ROS child processes and toy longitudinal plant; not Gazebo or hardware',
                           measurement_quality='monitor receipt times include DDS/scheduling; no latency bound '
                           'or real braking guarantee; unavailable killed-source zeros remain null',
                           fixture='each scan has ego-local rectangular walls x +/-6m, y +/-0.425m; '
                           'sequential rays use toy motion history; steering is not integrated; no fixed world course',
                           timing=dict(dt_sim_s=DT, sleep_wall_s=sleep_s, scan_hz=10, imu_hz=200,
                                       encoder_hz=100, rgb_hz=10), ros_domain_id=os.environ['ROS_DOMAIN_ID'],
                           scenario=scenario, cycles=cycles, mcu_deadline_diagnostics=[])
        self.children = OwnedProcesses(output, self.report)
        self.node = rclpy.create_node('pipeline_fault_validator',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        topics = dict(clock='/clock', image='/camera/color/image_raw', scan='/scan',
                      imu='/imu/data', motor='/drive_motor/encoder')
        self.publishers = {key: self.node.create_publisher(self.types[key], topic,
                          qos_profile_sensor_data if key in ('scan', 'imu') else 100)
                           for key, topic in topics.items()}
        for key, topic in [('autonomy', '/autonomy/status'), ('safety', '/safety/status'),
                           ('mcu', '/actuation/status'), ('sink', '/actuation/output_status')]:
            self.node.create_subscription(String, topic, lambda m, key=key: self.on_status(key, m), 100)
        for key, topic in [('drive', '/drive'), ('safe', '/drive/safe')]:
            self.node.create_subscription(AckermannDriveStamped, topic,
                lambda m, key=key: self.monitor.receive(key, m.drive.speed, self.t,
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 100)
        from rclpy.qos import DurabilityPolicy, QoSProfile
        # 가상 MCU가 첫 control_deadline_missed에만 한 번 발행하는 진단. 판정 입력이 아니다.
        self.node.create_subscription(String, '/actuation/diagnostics', lambda m: self.report[
            'mcu_deadline_diagnostics'].append(dict(received_wall_s=time.monotonic(), received_sim_s=self.t,
                                                    value=json.loads(m.data))),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Twist, '/sim/cmd_vel',
                                      lambda m: self.monitor.receive('applied', m.linear.x, self.t), 100)
        self.clients = {name: self.node.create_client(Trigger, topic) for name, topic in
                        [('arm', '/actuation/arm'), ('clear_stop', '/actuation/clear_stop'),
                         ('stop', '/actuation/stop'), ('reset', '/autonomy/reset')]}
        self.color, self.images, self.held_stamp = 'green', True, None
        self.rgb = {name: signal_rgb(name) for name in ('red', 'green')}
        self.last = {key: -math.inf for key in ('scan', 'image', 'motor')}
        self.last_counts = None
        self.last_motor_t = None

    def on_status(self, key, message):
        value = json.loads(message.data)
        stamp = value.get('sim_time_s') if key == 'autonomy' else (
            value.get('now_us', 0) * 1e-6 if key in ('mcu', 'sink') else None)
        self.monitor.receive(key, value, self.t, stamp)

    def data(self, key):
        return self.monitor.latest.get(key, {}).get('value', {})

    def fresh(self, key, after, predicate):
        return self.monitor.fresh(key, after, predicate, time.monotonic(), self.t,
                                  max_age=.35 if key == 'autonomy' else .25)

    def stamp(self, msg, seconds):
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp_parts(seconds)
        return msg

    def position_at(self, stamp):
        history = list(self.history)
        index = bisect_right([s for s, _ in history], stamp) - 1
        if index < 0:
            return history[0][1]
        if index + 1 >= len(history):
            return history[-1][1]
        a, b = history[index], history[index + 1]
        return a[1] + (b[1] - a[1]) * (stamp - a[0]) / (b[0] - a[0])

    def tick(self):
        if time.monotonic() > self.deadline:
            raise TimeoutError('total wall execution cap exceeded')
        self.children.check()
        self.t = round(self.t + DT, 9)
        self.plant.step(float(self.data('applied') or 0.))
        self.history.append((self.t, self.plant.distance))
        clock = self.types['clock']()
        clock.clock.sec, clock.clock.nanosec = stamp_parts(self.t)
        self.publishers['clock'].publish(clock)
        imu = self.stamp(self.types['imu'](), self.t)
        self.publishers['imu'].publish(imu)
        if self.t - self.last['motor'] >= .01 - 1e-8:
            counts = round(self.plant.distance / self.spec.mean_wheel_travel_m_per_count)
            motor = self.stamp(self.types['motor'](), self.t)
            motor.name = ['drive_motor_shaft']
            motor.position = [self.spec.motor_angle_from_counts(counts)]
            motor.velocity = [math.nan if self.last_counts is None else
                              self.spec.motor_rad_s_from_counts(counts - self.last_counts,
                                                               self.t - self.last_motor_t)]
            self.publishers['motor'].publish(motor)
            self.last_counts, self.last_motor_t, self.last['motor'] = counts, self.t, self.t
        if self.t - self.last['scan'] >= .1 - 1e-8:
            scan = self.stamp(self.types['scan'](), self.t - .1)
            scan.header.frame_id = 'laser_frame'
            scan.angle_min, scan.angle_increment = -math.pi, math.tau / 500
            scan.angle_max = scan.angle_min + 499 * scan.angle_increment
            scan.range_min, scan.range_max = .05, 12.
            scan.time_increment, scan.scan_time = .1 / 500, .1
            scan.ranges = [corridor_range(scan.angle_min + i * scan.angle_increment,
                .06 + self.position_at(self.t - .1 + i * .1 / 500) - self.plant.distance)
                           for i in range(500)]
            self.publishers['scan'].publish(scan)
            self.last['scan'] = self.t
        if self.images and self.t - self.last['image'] >= .1 - 1e-8:
            image = self.stamp(self.types['image'](), self.t if self.held_stamp is None else self.held_stamp)
            image.header.frame_id = 'camera_color_optical_frame'
            image.height, image.width, image.step, image.encoding = 480, 848, 848 * 3, 'rgb8'
            image.data = self.rgb[self.color]
            self.publishers['image'].publish(image)
            self.last['image'] = self.t
        for _ in range(24):
            self.rclpy.spin_once(self.node, timeout_sec=0)
        self.monitor.check_zero_watch(time.monotonic(), self.t)
        time.sleep(self.sleep_s)

    def pump(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.tick()

    def wait(self, predicate, label, timeout=15.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.tick()
            if predicate():
                return
        raise AssertionError('timeout: ' + label + '; latest=' + json.dumps(
            {k: r['value'] for k, r in self.monitor.latest.items()}))

    def call(self, name):
        client = self.clients[name]
        self.wait(client.service_is_ready, name + ' service ready')
        baseline = self.monitor.sequence
        if name == 'arm':
            self.wait(lambda: self.fresh('mcu', baseline, lambda v: v['output_ready']), 'fresh output ready')
        future = client.call_async(self.Trigger.Request())
        self.wait(future.done, name + ' ACK')
        result = future.result()
        self.monitor.events.append(dict(kind='service', name=name, wall_s=time.monotonic(),
                                        sim_s=self.t, success=result.success, message=result.message))
        if not result.success:
            raise AssertionError(name + ': ' + result.message)

    def mark(self, name):
        row = dict(name=name, passed=False, injected_wall_s=time.monotonic(), injected_sim_s=self.t,
                   sequence=self.monitor.sequence, position_at_injection_m=self.plant.distance,
                   first_zero={}, last_positive={})
        for key in ('drive', 'safe', 'applied'):
            row['last_positive'][key] = next((r for r in reversed(self.monitor.rows)
                                             if r['topic'] == key and r['value'] > 0), None)
        self.report['cases'].append(row)
        return row

    def finish(self, case, *, expected, duration=.65, require_stop=False):
        self.pump(duration)
        tail_seq = self.monitor.sequence
        self.wait(lambda: all(self.fresh(key, tail_seq, test) for key, test in expected.items())
                  and (not require_stop or self.stopped(tail_seq)),
                  case['name'] + ' fresh tail')
        self.monitor.check_zero_watch(time.monotonic(), self.t)
        if require_stop and self.monitor.zero_watch is not None and any(
                count == 0 for count in self.monitor.zero_watch['counts'].values()):
            raise AssertionError('zero hold did not observe every output topic')
        case.update(passed=True, final_sim_s=self.t, final_wall_s=time.monotonic(),
                    model_speed_mps=self.plant.speed,
                    model_displacement_m=self.plant.distance - case['position_at_injection_m'],
                    final={key: self.monitor.latest[key] for key in expected})
        if require_stop:
            case['final'].update({key: self.monitor.latest[key] for key in ZERO_TOPICS})
        self.capture_zeros(case)
        print(case['name'] + ': passed', flush=True)

    def capture_zeros(self, case):
        case['first_zero_semantics'] = 'post-injection source time when present; otherwise monitor receipt only'
        case['first_zero_receipt_only'] = {}
        case['excluded_pre_injection_source_zero'] = {}
        for key in ('drive', 'safe', 'applied', 'autonomy', 'safety', 'mcu', 'sink'):
            def is_zero(row):
                value = row['value']
                if not isinstance(value, dict):
                    return value == 0.
                return value.get('speed_command_mps', value.get('safe_speed_mps',
                    value.get('target_speed_mps', value.get('speed_mps')))) == 0.
            candidates = [r for r in self.monitor.rows
                if r['topic'] == key and r['sequence'] > case['sequence']
                and r['received_wall_s'] >= case['injected_wall_s'] and is_zero(r)]
            case['first_zero'][key] = next((r for r in candidates if r['source_sim_s'] is not None
                and r['source_sim_s'] >= case['injected_sim_s']), None)
            case['first_zero_receipt_only'][key] = next((r for r in candidates
                if r['source_sim_s'] is None), None)
            case['excluded_pre_injection_source_zero'][key] = next((r for r in candidates
                if r['source_sim_s'] is not None and r['source_sim_s'] < case['injected_sim_s']), None)

    def stopped(self, after):
        return (abs(self.plant.speed) < .01 and
                self.fresh('applied', after, lambda v: v == 0.) and
                self.fresh('sink', after, lambda v: v['speed_mps'] == 0.) and
                self.fresh('mcu', after, lambda v: v['target_speed_mps'] == 0.))

    def stop_case(self, name, action, expected, *, no_resume=False):
        if no_resume and not self.stopped(0):
            raise AssertionError(name + ': no-resume requires fresh initial stopped state')
        case = self.mark(name)
        case['no_resume'] = no_resume
        if no_resume:
            case['initial_stopped_outputs'] = {key: self.monitor.latest[key] for key in ZERO_TOPICS}
            self.monitor.begin_zero_watch(case)
        action()
        self.monitor.check_zero_watch(time.monotonic(), self.t)
        self.wait(lambda: self.stopped(case['sequence']) and all(
            self.fresh(key, case['sequence'], test) for key, test in expected.items()), name + ' stop')
        case['stable_model_stop_start'] = dict(wall_s=time.monotonic(), sim_s=self.t,
                                             position_m=self.plant.distance)
        stationary_start = self.plant.distance
        seq = self.monitor.sequence
        if not no_resume:
            hold = dict(sequence=seq, name=name + ' stable stop')
            self.monitor.begin_zero_watch(hold)
        self.pump(.7)
        stationary_rows = [r for r in self.monitor.rows if r['sequence'] > seq
                           and r['topic'] in ('applied', 'sink', 'mcu')]
        if (not stationary_rows or any((r['value'] if r['topic'] == 'applied' else
                r['value']['speed_mps' if r['topic'] == 'sink' else 'target_speed_mps']) != 0.
                for r in stationary_rows) or abs(self.plant.distance - stationary_start) > .005):
            raise AssertionError(name + ': stopped output did not remain zero')
        case['stable_stop_displacement_m'] = self.plant.distance - stationary_start
        self.finish(case, expected={'applied': lambda v: v == 0., **expected}, duration=0., require_stop=True)
        if not no_resume:
            case['stable_zero_observation'] = hold['zero_observation']
            case['zero_receipt_bounds'] = hold['zero_receipt_bounds']
        self.monitor.end_zero_watch()
        return case

    def moving(self, name):
        case = self.mark(name)
        self.wait(lambda: self.plant.speed > .4 and all(self.fresh(key, case['sequence'],
                  lambda v: v > .4) for key in ('drive', 'safe', 'applied')), name)
        self.finish(case, expected={key: lambda v: v > .4 for key in ('drive', 'safe', 'applied')}, duration=.1)
        return case

    def red_green(self):
        self.images, self.held_stamp, self.color = True, None, 'red'
        baseline = self.monitor.sequence
        self.wait(lambda: self.fresh('autonomy', baseline, lambda v: v['signal'] == 'red'), 'actual red pixels')
        self.pump(.8)
        self.color = 'green'
        self.moving('fresh_red_to_green_recovery')

    def restart_controller(self):
        def restart():
            self.children.stop('controller')
            self.color, self.images, self.held_stamp = 'green', True, None
            self.children.start('controller')
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline,
                lambda v: not v['started'] and v['vision']['stats']['accepted'] >= 3),
                'restart accepts green pixels without start permission')
            self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
        self.stop_case('controller_restart_green_only_no_start', restart,
                       {'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.}, no_resume=True)
        self.red_green()

    def graph_snapshot(self):
        expected_publishers = {'/drive': 'local_pursuit', '/drive/safe': 'lidar_safety',
                               '/sim/cmd_vel': 'guarded_sim_actuator'}
        audit = dict(publishers={}, subscriptions={})
        for topic, owner in expected_publishers.items():
            audit['publishers'][topic] = [dict(node=info.node_name, namespace=info.node_namespace,
                gid=list(info.endpoint_gid)) for info in self.node.get_publishers_info_by_topic(topic)]
        for topic in ('/clock', '/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data'):
            audit['publishers'][topic] = [dict(node=info.node_name, namespace=info.node_namespace,
                gid=list(info.endpoint_gid)) for info in self.node.get_publishers_info_by_topic(topic)]
        sensor_topics = {'/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data',
                         '/clock', '/parameter_events'}
        for name, allowed in [('local_pursuit', sensor_topics),
                              ('lidar_safety', sensor_topics - {'/camera/color/image_raw'} | {'/drive'})]:
            topics = {t for t, _ in self.node.get_subscriber_names_and_types_by_node(name, '/')}
            audit['subscriptions'][name] = sorted(topics)
            required = {'/scan', '/clock', '/drive_motor/encoder', '/imu/data'} | (
                {'/camera/color/image_raw'} if name == 'local_pursuit' else {'/drive'})
            audit.setdefault('input_checks', {})[name] = not topics - allowed and required <= topics
        expected_publishers.update({topic: 'pipeline_fault_validator' for topic in
            ('/clock', '/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data')})
        audit['valid'] = all([entry['node'] for entry in audit['publishers'][topic]] == [owner]
            and all(entry['namespace'] == '/' for entry in audit['publishers'][topic])
            for topic, owner in expected_publishers.items()) and all(audit['input_checks'].values())
        return audit

    def graph_audit(self, convergence_wall_s=0.):
        observation = dict(started_wall_s=time.monotonic(), convergence_wall_cap_s=convergence_wall_s,
                           changes=[], passed=False)
        self.report.setdefault('graph_audits', []).append(observation)
        previous = None
        until = time.monotonic() + convergence_wall_s
        while True:
            audit = self.graph_snapshot()
            signature = json.dumps(audit, sort_keys=True)
            if signature != previous:
                observation['changes'].append(dict(wall_s=time.monotonic(), sim_s=self.t, graph=audit,
                    owned_process_exits=[dict(role=owned['record']['role'], pid=owned['record']['pid'],
                        exit_code=owned['child'].poll(), expected_exit=owned['record']['expected_exit'])
                        for owned in self.children.children]))
                previous = signature
            if audit['valid']:
                observation.update(passed=True, final_wall_s=time.monotonic(), final_sim_s=self.t)
                self.report['graph_audit'] = audit
                return
            if time.monotonic() >= until:
                raise AssertionError('graph did not converge to exact single expected publishers: ' + signature)
            self.pump(.1)

    def run(self):
        for role in ('host', 'mcu', 'sink', 'safety', 'controller'):
            self.children.start(role)
        baseline = self.monitor.sequence
        self.wait(lambda: self.fresh('mcu', baseline, lambda v: v['feedback_valid']) and
                  self.fresh('autonomy', baseline, lambda v: not v['started'] and
                             v['vision']['stats']['accepted'] >= 3), 'pipeline startup')
        self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
        self.graph_audit()
        self.wait(lambda: self.stopped(0), 'fresh startup zero outputs')
        self.stop_case('boot_disarmed', lambda: None, {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
        self.stop_case('armed_green_only_cannot_start', lambda: self.call('arm'),
                       {'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.}, no_resume=True)
        self.red_green()
        self.stop_case('rgb_drop_sensor_stop', lambda: setattr(self, 'images', False),
                       {'autonomy': lambda v: v['state'] == 'SENSOR_STOP',
                        'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        self.images = True
        self.moving('rgb_current_restore_automatic_resume')
        self.held_stamp = self.last['image']
        self.stop_case('rgb_old_identical_stamp_rejected', lambda: None,
                       {'autonomy': lambda v: v['state'] == 'SENSOR_STOP',
                        'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        self.held_stamp = None
        accepted = self.moving('fresh_stamped_identical_green_accepted')
        accepted['interpretation'] = 'known observability limit; not successful stuck-camera detection'
        if self.scenario == 'restart_repro':
            self.run_restart_repro()
            return
        for sig in (signal.SIGSTOP, signal.SIGKILL):
            self.vision_stop_restart_cycle(sig)
        for sig in (signal.SIGSTOP, signal.SIGKILL):
            pid = self.data('autonomy')['vision']['worker_pid']
            self.children.register_vision(pid)
            self.stop_case('controller_' + sig.name + ('_owned_group' if sig == signal.SIGKILL else '') + '_safety_stale_stop',
                           lambda sig=sig: self.children.send('controller', sig),
                           {'safety': lambda v: v['reason'] == 'stale_input', 'safe': lambda v: v == 0.})
            if sig == signal.SIGSTOP:
                self.children.send('controller', signal.SIGCONT, phase='recovery')
                self.moving('controller_SIGCONT_automatic_resume')
            else:
                self.restart_controller()
        for sig in (signal.SIGSTOP, signal.SIGKILL):
            self.stop_case('safety_' + sig.name + '_command_expired_latch',
                           lambda sig=sig: self.children.send('safety', sig),
                           {'mcu': lambda v: v['state'] == 'FAULT_LATCHED' and v['reason'] == 'command_expired',
                            'sink': lambda v: v['blocked']})
            restore_safety = (lambda: self.children.send('safety', signal.SIGCONT, phase='recovery')) if sig == signal.SIGSTOP else (
                lambda: self.children.start('safety'))
            self.stop_case('safety_' + sig.name + '_restore_no_auto_move', restore_safety,
                           {'mcu': lambda v: v['state'] == 'FAULT_LATCHED', 'safe': lambda v: v > .4}, no_resume=True)
            self.stop_case('safety_' + sig.name + '_clear_only_disarmed', lambda: self.call('clear_stop'),
                           {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
            self.call('arm')
            self.moving('safety_' + sig.name + '_explicit_rearm_recovers')
        self.final_stop()

    def vision_stop_restart_cycle(self, sig):
        pid = self.data('autonomy')['vision']['worker_pid']
        owned = self.children.register_vision(pid)
        self.stop_case('vision_' + sig.name + '_fault_latched',
            lambda: self.children.send('controller', sig, vision=owned),
            {'autonomy': lambda v: v['state'] == 'SENSOR_STOP' and bool(v['vision']['fault']),
             'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        def restore_old_images():
            self.held_stamp = self.last['image']
            if sig == signal.SIGSTOP:
                self.children.send('controller', signal.SIGCONT, vision=owned, phase='recovery')
        self.stop_case('vision_' + sig.name + '_old_repeated_images_no_resume', restore_old_images,
                       {'autonomy': lambda v: bool(v['vision']['fault'])}, no_resume=True)
        self.stop_case('vision_' + sig.name + '_fresh_images_no_resume', lambda: setattr(self, 'held_stamp', None),
                       {'autonomy': lambda v: bool(v['vision']['fault'])}, no_resume=True)
        self.stop_case('vision_' + sig.name + '_reset_no_resume', lambda: self.call('reset'),
                       {'autonomy': lambda v: bool(v['vision']['fault']) and not v['started']}, no_resume=True)
        self.restart_controller()

    def final_stop(self):
        self.stop_case('final_explicit_software_stop', lambda: self.call('stop'),
                       {'mcu': lambda v: v['state'] == 'ESTOP_LATCHED' and v['reason'] == 'software_stop',
                        'sink': lambda v: v['blocked']})
        final_graph_case = self.stop_case('final_graph_convergence_stays_stopped',
            lambda: self.graph_audit(convergence_wall_s=40.),
            {'mcu': lambda v: v['state'] == 'ESTOP_LATCHED', 'sink': lambda v: v['blocked']}, no_resume=True)
        final_graph_case['interpretation'] = 'observed endpoint convergence only; no asserted DDS removal cause'

    def run_restart_repro(self):
        """ros_v2 실패 구간(영상 SIGSTOP 고정→reset→제어기 재시작→빨강/초록)만 반복한다."""
        for cycle in range(self.cycles):
            self.report.setdefault('cycle_starts', []).append(dict(cycle=cycle + 1, sim_s=self.t,
                                                                  wall_s=time.monotonic()))
            self.vision_stop_restart_cycle(signal.SIGSTOP)
        self.final_stop()

    def record_failure_state(self):
        state = dict(wall_s=time.monotonic(), sim_s=self.t,
                     latest={key: row['value'] for key, row in self.monitor.latest.items()})
        try:
            state['graph'] = self.graph_snapshot()
        except Exception as error:
            state['graph_error'] = repr(error)
        state['owned_processes'] = [dict(role=o['record']['role'], pid=o['record']['pid'],
                                         exit_code=o['child'].poll()) for o in self.children.children]
        self.report['failure_state'] = state

    def snapshot_sources(self):
        names = ['scripts/validate_pipeline_faults_ros.py', *SOURCES, *PROVENANCE]
        self.report['source_snapshot_scope'] = dict(runtime_modules=SOURCES,
            validator='scripts/validate_pipeline_faults_ros.py', provenance_only=PROVENANCE,
            provenance_interpretation='test and configuration metadata; vehicle/track files are not loaded by this fixture')
        self.report['source_sha256'] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                        for name in names}
        with zipfile.ZipFile(self.output / 'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
            for name in names:
                archive.write(ROOT / name, name)

    def check_runtime_sources(self):
        valid = True
        for owned in self.children.children:
            record = owned['record']
            lines = (self.output / record['log']).read_text(errors='replace').splitlines()
            row = next((line.split('=', 1)[1] for line in lines
                        if line.startswith('RUNTIME_SOURCE_MANIFEST=')), None)
            record['runtime_sources'] = json.loads(row) if row else None
            if not row:
                valid = False
                continue
            expected = {f'{package}.{name}' for package, names in MODULES.items() for name in names}
            if set(record['runtime_sources']) != expected:
                valid = False
                record['runtime_manifest_modules_mismatch'] = True
            for module, info in record['runtime_sources'].items():
                package, name = module.split('.')
                relative = f'src/{package}/{package}/{name}.py'
                if info['sha256'] != self.report['source_sha256'][relative]:
                    valid = False
                    record.setdefault('runtime_hash_mismatch', []).append(module)
        return valid

    def audit_runtime_sources_safely(self):
        try:
            return self.check_runtime_sources()
        except Exception as error:
            self.report['runtime_source_error'] = repr(error)
            return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-cap', type=float, default=480.)
    parser.add_argument('--sleep-wall', type=float, default=.005)
    parser.add_argument('--scenario', choices=('full', 'restart_repro'), default='full',
                        help='restart_repro: ros_v2 failure segment only; not the 34-condition validation')
    parser.add_argument('--cycles', type=int, default=1, choices=(1, 2, 3))
    args = parser.parse_args()
    if os.name != 'posix' or not Path('/proc').is_dir():
        parser.error('Linux ROS environment with /proc is required')
    if not 300 <= args.wall_cap <= 600 or not .005 <= args.sleep_wall <= .01:
        parser.error('wall-cap must be 300..600; sleep-wall must be .005..01')
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts') or output == ROOT / 'artifacts':
        parser.error('output must be a fresh subdirectory under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(100 + os.getpid() % 70),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    import rclpy
    rclpy.init()
    validator = None
    try:
        validator = PipelineValidator(output, args.wall_cap, args.sleep_wall, args.scenario, args.cycles)
        validator.snapshot_sources()
        validator.run()
        validator.report['passed'] = True
    except Exception as error:
        if validator:
            validator.report['error'] = repr(error)
            validator.record_failure_state()
            for case in validator.report['cases']:
                if not case['passed']:
                    validator.capture_zeros(case)
                    case.update(error=repr(error), failed_wall_s=time.monotonic(), failed_sim_s=validator.t,
                                model_speed_mps=validator.plant.speed,
                                model_displacement_m=validator.plant.distance - case['position_at_injection_m'])
        else:
            (output / 'report.json').write_text(json.dumps(dict(passed=False, error=repr(error))), encoding='utf-8')
    finally:
        if validator:
            try:
                clean = validator.children.cleanup()
            except Exception as error:
                clean = False
                validator.report['cleanup_audit_error'] = repr(error)
            hashes = validator.audit_runtime_sources_safely()
            validator.report['passed'] = validator.report['passed'] and clean and hashes
            validator.report['runtime_source_hashes_match'] = hashes
            (output / 'events.json').write_text(json.dumps(validator.monitor.events, ensure_ascii=False,
                indent=2, allow_nan=False), encoding='utf-8')
            (output / 'report.json').write_text(json.dumps(validator.report, ensure_ascii=False,
                indent=2, allow_nan=False), encoding='utf-8')
            validator.node.destroy_node()
        rclpy.shutdown()
    passed = bool(validator and validator.report['passed'])
    print(json.dumps(dict(passed=passed, output=str(output))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
