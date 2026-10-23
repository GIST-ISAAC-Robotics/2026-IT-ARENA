#!/usr/bin/env python3
"""G1-b: 공식 코스 Gazebo 상대차 + 순차 C1 + 독립 보호층 + 가상 MCU 저속 폐루프 평가기.

상대차는 평가 전용 정적 모델을 실행 사본 월드에만 넣고, 평가기가 정답 일정으로
set_pose한다. 제어기는 실제 렌더링된 순차 광선으로만 상대차를 본다. actor 일정·정답
자세·중심선·신호 상태는 평가·actor 시작 기준에만 사용하며 제어 입력으로 보내지 않는다.
판정선은 config/tests/g1_traffic_v1.json에 시험 전에 등록했다.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from shapely.strtree import STRtree

from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch
from validate_basic_autonomy import (DEMO_WORLD, REPO, add_rate_metrics, configured_feedback_topic,
                                     early_start_detected, start_demo_process)
from validate_facility_visibility import yaw_of
import g1_traffic_eval as ev
from g1_lidar_stress import load_test_profile, audit_injection, require_injection_evidence

CONFIG = REPO/'config/tests/g1_traffic_v1.json'
WORLD_NAME = 'it_arena_track'
SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'
CONTACT_TOPIC = '/eval/g1b_actor/contact'
CONTACT_PROBE_REPORT = REPO/'artifacts/validation/2026-10-07/g1b_contact_probe_v1/report.json'
SOURCE_FILES = [
    'scripts/g1_lidar_stress.py',
    'scripts/validate_g1_traffic.py', 'scripts/g1_traffic_eval.py', 'scripts/validate_basic_autonomy.py',
    'scripts/lidar_shutdown.py', 'scripts/audit_local_pursuit_road.py', 'scripts/probe_static_actor_contact.py',
    'config/tests/g1_traffic_v1.json', 'config/tests/g1b_r0_sha256.json',
    'src/arena_gazebo/worlds/it_arena_official/world.sdf', 'src/arena_gazebo/worlds/it_arena_official/scene.json',
    'src/arena_gazebo/worlds/it_arena_official/centerline.csv',
    'src/arena_description/config/vehicle.yaml', 'src/arena_description/config/b0183_c1.yaml',
    'src/arena_description/models/arena_car/model.sdf.xacro',
    'src/arena_bringup/launch/simulation.launch.py', 'src/arena_bringup/launch/demo.launch.py',
    'src/arena_bringup/launch/local_pursuit.launch.py', 'src/arena_bringup/launch/g1_follow.launch.py',
    'src/arena_bringup/config/b0183_imu_bridge.yaml',
    *[f'src/arena_autonomy/arena_autonomy/{name}.py' for name in (
        'core', 'wall_follow', 'local_path', 'local_pursuit', 'local_follow', 'scene_motion', 'local_scene',
        'vision_process', 'lidar_safety', 'lidar_observation', 'lidar_motion', 'lidar_motion_ros', 'follow_trace',
        'boundary_evidence', 'corner_evidence', 'corner_distance', 'corner_corridor', 'corridor_observer', 'occupancy_timing', 'race_start',
        'observation_support', 'observation_lateral', 'linear_observation')],
    *[f'src/arena_vehicle_interface/arena_vehicle_interface/{name}.py' for name in (
        'rotating_lidar', 'lidar_impairment', 'sim_wheel_encoder', 'ackermann_to_twist', 'timing_probe', 'queued_log',
        'actuation_ros', 'applied_guard', 'actuation_contract', 'actuation_wire')],
    'src/arena_gazebo/src/single_motor_drive.cpp', 'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp',
    'src/arena_gazebo/scripts/traffic_light_controller.py',
]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


ZERO_SPEED_MODES = ('legacy', 'reset_integral', 'no_lag', 'passive_damping')
PHYSICS_STEPS = ('0.0005', '0.00025')


def drivetrain_evaluation_arguments(zero_speed_mode='legacy', physics_step=None):
    """평가 전용 구동 대조 인자. 기본값은 빈 목록이라 기존 G1 실행 인자와 같다."""
    if zero_speed_mode not in ZERO_SPEED_MODES or physics_step not in (None, *PHYSICS_STEPS):
        raise ValueError('unsupported drivetrain evaluation option')
    extra = [] if zero_speed_mode == 'legacy' else [f'evaluation_zero_speed_mode:={zero_speed_mode}']
    return extra+([f'evaluation_physics_step:={physics_step}'] if physics_step else [])


def load_config(path=CONFIG):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if config.get('schema_version') != 1:
        raise ValueError('unsupported G1-b config')
    names = [case['name'] for case in config['cases']]
    if len(set(names)) != len(names):
        raise ValueError('duplicate case')
    for case in config['cases']:
        ev.validate_schedule(case['schedule'], case['duration_s'])
        lateral = actor_lateral_offset(config, case)
        if not math.isfinite(lateral) or abs(lateral) > .45:
            raise ValueError('invalid actor lateral offset')
        if lateral and case.get('route_kind') != 'side_grass_straight':
            raise ValueError('lateral actor requires registered side-grass route')
    if set(config['seeds']) != {'0', '1', '2'}:
        raise ValueError('three fixed seeds are required')
    return config


def case_named(config, name):
    for case in config['cases']:
        if case['name'] == name:
            return case
    raise ValueError(f'unknown case {name}')


def load_centerline(world=DEMO_WORLD):
    route = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
    scene = json.loads((world/'scene.json').read_text(encoding='utf-8'))
    return ev.Centerline(route['x_m'], route['y_m'], route['s_m'], scene['track']['lap_length_m']), scene


def actor_start_s(config, scene, seed, case=None):
    slot = {int(s['index']): s for s in scene['starting_grid']['slots']}[config['launch']['grid_slot']]
    gap = (case or {}).get('actor_start_center_gap_m', config['actor']['start_center_gap_m'])
    return slot['s_m']+gap+config['seeds'][str(seed)]['actor_start_offset_m']


def check_straight(config, centerline, case, s0):
    """actor 마지막 위치가 첫 코너 전에 머무는지 시험 전 확인한다."""
    end = centerline.unwrap(s0+ev.actor_travel_m(case['schedule'], case['duration_s']), s0)
    corner = centerline.unwrap(config['first_corner_s_m'], s0)
    if corner < s0:
        corner += centerline.lap
    if end > corner-1.0:
        raise ValueError(f'actor leaves the registered straight section: end s={end:.3f}, corner s={corner:.3f}')
    return {'actor_start_s_m': s0, 'actor_end_s_m': end, 'first_corner_s_m': corner}


def check_route(config, centerline, case, s0, road=None, obstacles=()):
    """확장 시험만 등록한 첫 코너까지 허용한다. 기존 직선 제한은 유지한다."""
    kind = case.get('route_kind', 'straight')
    if kind == 'straight':
        return {'kind': kind, **check_straight(config, centerline, case, s0)}
    if kind == 'side_grass_straight':
        straight = check_straight(config, centerline, case, s0)
        lateral = actor_lateral_offset(config, case)
        if road is None or not lateral:
            raise ValueError('side-grass route needs road and a nonzero offset')
        actor = config['actor']
        travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
        samples = np.linspace(s0, s0+travel, math.ceil(travel/.01)+1)
        tree = STRtree(obstacles)
        corridor = road.buffer(.20)  # 현재 공식 직선의 20 cm 잔디 폭. 차량 도로 기준은 바꾸지 않음.
        for s in samples:
            pose = actor_pose_at(centerline, s, [], 0., lateral)
            footprint = rectangle(*pose, actor['length_m'], actor['width_m'])
            if (footprint.intersection(road).area > 1e-6 or footprint.difference(corridor).area > 1e-6 or
                    any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))):
                raise ValueError(f'side actor not wholly off-road inside wall corridor at s={s:.4f}')
        return {'kind': kind, **straight, 'lateral_offset_m': lateral,
                'actor_offroad_samples': len(samples), 'maximum_sample_step_m': .01,
                'scope': '잔디 쪽 사고/정지차 가림 시험. 정상 병렬 주행이나 잔디 사용 허가가 아님.'}
    if kind != 'first_curve':
        raise ValueError(f'unknown route kind: {kind}')
    rules = config['acceptance'][case['name']]
    travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
    end = s0+travel
    exit_s = centerline.unwrap(rules['required_exit_s_m'], s0)
    if exit_s < s0:
        exit_s += centerline.lap
    limit = centerline.unwrap(rules['maximum_actor_s_m'], s0)
    if limit < s0:
        limit += centerline.lap
    if not (s0 < exit_s <= end <= limit) or road is None:
        raise ValueError('curve route not covered, exceeds registered extent, or road is missing')
    actor = config['actor']
    samples = np.linspace(s0, end, math.ceil(travel/.01)+1)
    tree = STRtree(obstacles)
    for s in samples:
        footprint = rectangle(*centerline.pose(s), actor['length_m'], actor['width_m'])
        if (footprint.difference(road).area > 1e-6 or
                any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))):
            raise ValueError(f'actor route footprint outside road or into wall at s={s:.4f}')
    return {'kind': kind, 'actor_start_s_m': s0, 'actor_end_s_m': end,
            'required_exit_s_m': exit_s, 'maximum_actor_s_m': limit,
            'actor_road_wall_samples': len(samples), 'maximum_sample_step_m': .01}


def source_paths(config_path, lidar_profile=None):
    """사용자 지정 시험 설정도 소스 사본에 포함하며 저장소 내부 경로만 허용한다."""
    relative = Path(config_path).resolve().relative_to(REPO).as_posix()
    extra = [Path(lidar_profile).resolve().relative_to(REPO).as_posix()] if lidar_profile else []
    return list(dict.fromkeys([*SOURCE_FILES, relative, *extra]))


def actor_lateral_offset(config, case):
    return float(case.get('actor_lateral_offset_m', config['actor'].get('lateral_offset_m', 0.)))


def actor_pose_at(centerline, s0, schedule, elapsed, lateral_offset=0.):
    x, y, yaw = centerline.pose(s0+ev.actor_travel_m(schedule, elapsed))
    return x-lateral_offset*math.sin(yaw), y+lateral_offset*math.cos(yaw), yaw


def actor_sdf(config, pose):
    actor = config['actor']
    x, y, yaw = pose
    z = actor['bottom_m']+actor['height_m']/2
    size = f"{actor['length_m']} {actor['width_m']} {actor['height_m']}"
    root = ET.Element('sdf', version='1.9')
    ET.SubElement(root, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    model = ET.SubElement(root, 'model', name=actor['model_name'])
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.9f}'
    link = ET.SubElement(model, 'link', name='body')
    collision = ET.SubElement(link, 'collision', name='collision')
    ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = size
    visual = ET.SubElement(link, 'visual', name='visual')
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = size
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = '.55 .08 .08 1'
    sensor = ET.SubElement(link, 'sensor', name='eval_contact', type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '200'
    contact = ET.SubElement(sensor, 'contact')
    ET.SubElement(contact, 'collision').text = 'collision'
    ET.SubElement(contact, 'topic').text = CONTACT_TOPIC
    publisher = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system',
                              name='gz::sim::systems::PosePublisher')
    for tag, value in (('publish_model_pose', 'true'), ('publish_link_pose', 'false'),
                       ('use_pose_vector_msg', 'true'), ('static_publisher', 'false'),
                       ('update_frequency', str(int(actor['pose_publish_hz'])))):
        ET.SubElement(publisher, tag).text = value
    return ET.tostring(root, encoding='unicode')


def stamp(message):
    return message.sec+message.nanosec*1e-9


class EvaluationCaptureGate:
    """시험 종료 후 자료 저장 동안 일부 transport 스트림만 길어지지 않게 한다."""
    def __init__(self):
        self.lock = threading.Lock()
        self.closed = False

    def wrap(self, handler):
        def accept(data):
            with self.lock:
                if not self.closed:
                    handler(data)
        return accept

    def close(self):
        with self.lock:
            self.closed = True


def json_reader(command, handler, log, audit):
    """gz topic JSON 출력의 평가 전용 수신기. 시험자 소유 프로세스 그룹."""
    process = subprocess.Popen(command, cwd=REPO, stdout=subprocess.PIPE, stderr=log, text=True,
                               start_new_session=True)

    def run():
        for line in process.stdout:
            try:
                data = json.loads(line)
            except (ValueError, TypeError) as error:
                audit['parse_errors'] += 1
                audit['last_error'] = f'{type(error).__name__}: {error}'
                continue
            audit['json_messages'] += 1
            try:
                handler(data)
                audit['handled_messages'] += 1
            except (ValueError, TypeError, KeyError) as error:
                audit['handler_errors'] += 1
                audit['last_error'] = f'{type(error).__name__}: {error}'
                continue
    threading.Thread(target=run, daemon=True).start()
    return process


def stop_owned(process, timeouts=((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5))):
    forced = []
    for sig, timeout in timeouts:
        if process.poll() is not None:
            break
        try:
            if sig != signal.SIGINT:
                forced.append(sig.name)
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            continue
    return forced


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True)
    parser.add_argument('--seed', type=int, choices=(0, 1, 2), required=True)
    parser.add_argument('--mode', choices=('g1', 'r0'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--lidar-test-profile', type=Path,
                        help='선택형 G1 시험용 거리오차/추가 지연. 기본 무주입, 취득 시각 유지')
    parser.add_argument('--record-braking-diagnostics', action='store_true',
                        help='평가 전용 바퀴/차체/토크 시계열 보존; 제어 및 판정은 변경하지 않음')
    parser.add_argument('--record-wheel-contacts', action='store_true',
                        help='실행 사본에 평가용 바퀴 접촉 센서를 추가하여 원시 접촉 메시지 보존')
    parser.add_argument('--record-follow-decisions', action='store_true',
                        help='선택형 G1 판단 시점의 스캔/운동 이력/출력을 기록; 제어 변경 없음')
    parser.add_argument('--boundary-evidence', choices=('connected', 'fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate'), default='connected',
                        help='G1 경계 증거 후보; 기본 connected 유지, fragmented는 명시 선택형')
    parser.add_argument('--occupancy-timing', choices=('oldest', 'per_ray_candidate'), default='oldest',
                        help='점별 취득 시각 팽창 후보. 기본 oldest 유지, 점/거리 여유/운동 상한 유지')
    parser.add_argument('--zero-speed-mode', choices=ZERO_SPEED_MODES, default='legacy',
                        help='평가 전용 0목표 구동 특성 대조. 기본 legacy는 실행 인자를 추가하지 않음')
    parser.add_argument('--physics-step', choices=PHYSICS_STEPS,
                        help='평가 전용 물리 step. 생략하면 월드 기본 1 ms')
    args = parser.parse_args()
    if args.occupancy_timing != 'oldest' and (args.mode != 'g1' or not args.record_follow_decisions):
        parser.error('--occupancy-timing candidate requires G1 with decision recording')
    if args.lidar_test_profile and (args.mode != 'g1' or not args.record_follow_decisions):
        parser.error('--lidar-test-profile requires G1 with decision recording')
    lidar_profile, lidar_requested = load_test_profile(args.lidar_test_profile, REPO)
    if args.record_follow_decisions and args.mode != 'g1':
        parser.error('--record-follow-decisions requires --mode g1')
    if args.boundary_evidence != 'connected' and (args.mode != 'g1' or not args.record_follow_decisions):
        parser.error('fragmented candidate requires G1 with decision recording')
    if (args.zero_speed_mode != 'legacy' or args.physics_step) and not args.record_braking_diagnostics:
        parser.error('구동 평가 옵션은 --record-braking-diagnostics로 적용 상태를 보존해야 합니다.')
    config = load_config(args.config)
    case = case_named(config, args.case)
    launch, acceptance, actor_cfg, limits = config['launch'], config['acceptance'], config['actor'], config['timeouts']
    if lidar_profile and (launch['lidar_rate_hz'] != 10 or launch['lidar_acquisition'] != 'sequential'):
        raise ValueError('registered G1 lidar stress requires sequential 10 Hz')
    output = args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('output must be under artifacts')
    centerline, scene = load_centerline()
    s0 = actor_start_s(config, scene, args.seed, case)
    lateral = actor_lateral_offset(config, case)
    route = check_route(config, centerline, case, s0, road_polygon(DEMO_WORLD/'world.sdf'),
                        obstacle_polygons(DEMO_WORLD/'world.sdf'))
    output.mkdir(parents=True, exist_ok=False)
    actor_path = output/'evaluation_actor.sdf'
    actor_path.write_text(actor_sdf(config, actor_pose_at(centerline, s0, case['schedule'], 0., lateral)), encoding='utf-8')
    sources = {rel: sha256(REPO/rel) for rel in source_paths(args.config, lidar_profile)}
    runtime_lidar_profile = None
    if lidar_profile:
        runtime_lidar_profile = output/'lidar_test_profile.json'
        runtime_lidar_profile.write_text(json.dumps(lidar_requested, indent=2), encoding='utf-8')
    with zipfile.ZipFile(output/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as bundle:
        for rel in sources:
            bundle.write(REPO/rel, rel)
        bundle.write(actor_path, 'evaluation_actor.sdf')
        if runtime_lidar_profile:
            bundle.write(runtime_lidar_profile, 'evaluation_lidar_test_profile.json')
    os.environ.update(ROS_DOMAIN_ID=str(150+os.getpid() % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}')
    # 위 파티션 이름은 lidar_shutdown.stop_gazebo_service의 시험 소유 확인 규칙과 같아야 한다.
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan, Imu
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from ros_gz_interfaces.srv import SetEntityPose
    from rosgraph_msgs.msg import Clock

    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'passed': False,
              'case': case['name'], 'seed': args.seed, 'mode': args.mode,
              'boundary_evidence_mode': args.boundary_evidence,
              'occupancy_timing_mode': args.occupancy_timing,
              'scope': config['scope'], 'config_sha256': sha256(args.config), 'source_sha256': sources,
              'actor_sdf_sha256': sha256(actor_path), 'route_check': route,
              'straight_section': route if route['kind'] == 'straight' else None,
              'actor_visible_section': actor_cfg['visible_section'], 'started_wall_monotonic': time.monotonic()}
    if runtime_lidar_profile:
        report['lidar_test_profile'] = {'path': str(lidar_profile.relative_to(REPO)),
            'requested': lidar_requested, 'runtime_sha256': sha256(runtime_lidar_profile),
            'scope': 'synthetic bounded radial error and publish delay, acquisition timestamps preserved'}
    if args.zero_speed_mode != 'legacy' or args.physics_step:
        # 평가 전용 구동 대조일 때만 기록한다. 기본 보고서 형식은 유지한다.
        report['drivetrain_evaluation'] = {'zero_speed_mode': args.zero_speed_mode,
                                           'physics_step_override_s': args.physics_step}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('g1b_traffic_validator')
    state, trace, scans, drive_stamps, acquisition_records = {}, [], [], [], []
    ego, actor, contacts, commands = [], [], [], []
    capture_gate = EvaluationCaptureGate()
    wheel_contacts = []
    reader_audit = {name: {'json_messages': 0, 'handled_messages': 0, 'parse_errors': 0,
                          'handler_errors': 0, 'last_error': None} for name in ('ego', 'contact')}
    if args.record_wheel_contacts:
        reader_audit['wheel_contact'] = dict(reader_audit['contact'])
    rejected_actor_times = 0
    rejected_ego_times = 0
    actuation_trace, actuation_diagnostics = [], []
    braking_diagnostics = []
    actor_frames = set()
    stop_requested = False

    def interrupted(*_):
        nonlocal stop_requested
        stop_requested = True
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)

    def remember(message, key):
        state[key] = message
        if key == 'drive':
            drive_stamps.append(stamp(message.header.stamp))

    def receive_clock(message):
        # STOP 요청 기준 시각. 평가기 노드의 최신 sim /clock을 정수 ns로만 보관하고 수신 횟수를 센다.
        state['clock_ns'] = int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
        state['clock_receipts'] = state.get('clock_receipts', 0)+1

    scan_stream = (output/'scans_received.jsonl').open('w', encoding='utf-8')
    status_stream = (output/'autonomy_status.jsonl').open('w', encoding='utf-8')

    def receive_scan(message):
        state['scan'] = message
        scans.append(stamp(message.header.stamp))
        # 사후 재현용 평가 기록. 제어 입력이 아니며 Git에는 해시만 남긴다.
        scan_stream.write(json.dumps({'stamp_s': scans[-1], 'time_increment': message.time_increment,
            'angle_min': message.angle_min, 'angle_increment': message.angle_increment,
            'ranges': [float(r) if math.isfinite(r) else None for r in message.ranges]})+'\n')

    def receive_status(message):
        status = json.loads(message.data)
        state['autonomy'] = status
        status_stream.write(json.dumps({k: v for k, v in status.items() if k not in ('text_log', 'vision')},
                                       ensure_ascii=False)+'\n')

    def receive_actor(message):
        nonlocal rejected_actor_times
        for transform in message.transforms:
            actor_frames.add((transform.header.frame_id, transform.child_frame_id))
            if transform.child_frame_id != actor_cfg['model_name'] or transform.header.frame_id != WORLD_NAME:
                continue
            when = stamp(transform.header.stamp)
            if actor and when <= actor[-1][0]:
                rejected_actor_times += 1
                continue
            q = transform.transform.rotation
            actor.append((when, float(transform.transform.translation.x), float(transform.transform.translation.y),
                          math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))

    def receive_actuation(message):
        status = json.loads(message.data)
        state['actuation'] = status
        actuation_trace.append(status)

    def receive_dynamics(message):
        status = json.loads(message.data)
        state['dynamics'] = status
        if args.record_braking_diagnostics:
            braking_diagnostics.append(status)

    reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [
        node.create_subscription(Image, '/camera/color/image_raw', lambda m: remember(m, 'rgb'), QoSProfile(depth=2)),
        node.create_subscription(Odometry, '/odom', lambda m: remember(m, 'odom'), 10),
        node.create_subscription(AckermannDriveStamped, '/drive', lambda m: remember(m, 'drive'), 10),
        node.create_subscription(String, '/autonomy/status', receive_status, 50),
        node.create_subscription(String, '/sim/traffic_light/state', lambda m: remember(json.loads(m.data), 'light'), 10),
        node.create_subscription(String, '/sim/drivetrain', receive_dynamics, 10),
        node.create_subscription(String, '/safety/status', lambda m: remember(json.loads(m.data), 'safety'), 10),
        node.create_subscription(AckermannDriveStamped, '/drive/safe', lambda m: remember(m, 'safe_drive'), 10),
        node.create_subscription(Imu, '/imu/data', lambda m: remember(m, 'imu'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data),
        node.create_subscription(String, '/sim/lidar_acquisition', lambda m: acquisition_records.append(json.loads(m.data)), 100),
        node.create_subscription(String, '/actuation/status', receive_actuation, 100),
        node.create_subscription(Clock, '/clock', receive_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(String, '/actuation/output_status', lambda m: remember(json.loads(m.data), 'actuation_output'), 100),
        node.create_subscription(String, '/actuation/diagnostics', lambda m: actuation_diagnostics.append(json.loads(m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(TFMessage, f"/model/{actor_cfg['model_name']}/pose", receive_actor, reliable),
    ]
    arm_service = node.create_client(Trigger, '/actuation/arm')
    mcu_stop_service = node.create_client(Trigger, '/actuation/stop')
    actor_client = node.create_client(SetEntityPose, SET_POSE_SERVICE)
    obstacles = obstacle_polygons(DEMO_WORLD/'world.sdf')
    tree = STRtree(obstacles)
    log_path = output/'simulation.log'
    log = log_path.open('w', encoding='utf-8')
    reader_log = (output/'readers.log').open('w', encoding='utf-8')
    trace_stream = (output/'trajectory.jsonl').open('w', encoding='utf-8')
    processes, readers = [], []
    arm_future = actor_future = None
    t_green = None
    outcome = {'abort': None}
    began = time.monotonic()

    def receive_ego(data):
        nonlocal rejected_ego_times
        header = data.get('header', {}).get('stamp', {})
        when = float(header.get('sec', 0))+float(header.get('nsec', 0))*1e-9
        for pose in data.get('pose', []):
            if pose.get('name') == 'arena_car':
                pose['time'] = when
                state['pose'] = pose
                if not ego or when > ego[-1][0]:
                    p = pose['position']
                    ego.append((when, float(p.get('x', 0.)), float(p.get('y', 0.)), yaw_of(pose['orientation'])))
                else:
                    rejected_ego_times += 1

    def receive_contact(data):
        header = data.get('header', {}).get('stamp', {})
        when = float(header.get('sec', 0))+float(header.get('nsec', 0))*1e-9
        pairs = [[c['collision1']['name'], c['collision2']['name']] for c in data.get('contact', [])]
        # 빈 Contacts도 수신 증거로 보존한다. 무접촉시 무발행과 구분한다.
        contacts.append({'t': when, 'pairs': pairs})

    def schedule_pose(t):
        elapsed = 0. if t_green is None else t-t_green
        x, y, _ = actor_pose_at(centerline, s0, case['schedule'], elapsed, lateral)
        return x, y

    def complete_actor_future():
        nonlocal actor_future
        if actor_future is not None and actor_future.done():
            response = actor_future.result()
            commands[-1].update(response_wall_s=time.monotonic()-began,
                                acknowledged=response is not None and response.success)
            actor_future = None
            if not commands[-1]['acknowledged']:
                raise RuntimeError('actor set_pose was not acknowledged')
        if actor_future is not None and time.monotonic()-commands[-1]['requested_wall_absolute_s'] > 8:
            raise TimeoutError('actor set_pose timeout')

    try:
        extra = [f'evaluation_actor_sdf:={actor_path}']
        if args.boundary_evidence != 'connected':
            extra.append(f'g1_boundary_evidence_mode:={args.boundary_evidence}')
        if args.record_follow_decisions:
            extra.append(f'g1_trace_path:={output / "follow_decisions.jsonl"}')
        if args.occupancy_timing != 'oldest':
            extra.append(f'g1_occupancy_timing_mode:={args.occupancy_timing}')
        if args.record_wheel_contacts:
            extra.append('evaluation_wheel_contacts:=true')
        extra += drivetrain_evaluation_arguments(args.zero_speed_mode, args.physics_step)
        processes.append(start_demo_process(
            log, launch['speed_profile'], launch['lidar_rate_hz'], True, launch['red_duration_s'],
            launch['autonomy_mode'], False, launch['lidar_acquisition'], launch['lidar_compensation'],
            launch['control_rate_hz'], launch['render_backend'], launch['sensor_wall_timeout_s'],
            launch['speed_bump'], launch['collision_detector'], str(output), launch['grid_slot'],
            str(runtime_lidar_profile) if runtime_lidar_profile else '', '',
            launch['actuation_mode'], False, args.mode == 'g1', extra))
        from ament_index_python.packages import get_package_prefix
        bridge_executable = Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/parameter_bridge'
        processes.append(subprocess.Popen([str(bridge_executable),
            f'{SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose',
            f"/model/{actor_cfg['model_name']}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            '--ros-args', '-r', '__node:=eval_actor_bridge',
            '-p', f"qos_overrides./model/{actor_cfg['model_name']}/pose.publisher.reliability:=reliable",
            '-p', f"qos_overrides./model/{actor_cfg['model_name']}/pose.publisher.depth:=200"],
            cwd=REPO, stdin=subprocess.DEVNULL, stdout=reader_log, stderr=reader_log, start_new_session=True))
        readers.append(json_reader(['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info', '--json-output'],
                                   capture_gate.wrap(receive_ego), reader_log, reader_audit['ego']))
        readers.append(json_reader(['gz', 'topic', '-e', '-t', CONTACT_TOPIC, '--json-output'],
                                   capture_gate.wrap(receive_contact), reader_log, reader_audit['contact']))
        if args.record_wheel_contacts:
            readers.append(json_reader(['gz', 'topic', '-e', '-t', '/eval/wheels/contact', '--json-output'],
                                       capture_gate.wrap(wheel_contacts.append), reader_log, reader_audit['wheel_contact']))
        last_record = -math.inf
        last_command_s = None
        first_pose_time = None
        signal_states = set()
        false_start = False
        wall_collision = 0
        progress = 0.
        last_s = None
        while True:
            rclpy.spin_once(node, timeout_sec=.005)
            if stop_requested:
                raise RuntimeError('interrupted')
            if any(p.poll() is not None for p in processes+readers):
                raise RuntimeError('owned process exited early')
            if time.monotonic()-began > limits['wall_timeout_s']:
                raise TimeoutError('wall timeout')
            complete_actor_future()
            if 'actuation' in state:
                status = state['actuation']
                if (arm_future is None and status['stationary_ready'] and status['output_ready']
                        and arm_service.service_is_ready() and status['now_us']-actuation_trace[0]['now_us'] > 1_000_000):
                    arm_future = arm_service.call_async(Trigger.Request())
                if arm_future is not None and arm_future.done() and not arm_future.result().success:
                    raise RuntimeError('MCU ARM refused: '+arm_future.result().message)
                if status['state'] in ('FAULT_LATCHED', 'ESTOP_LATCHED'):
                    outcome['abort'] = 'mcu_latched_before_stop: '+str(status['reason'])
                    break
            required = {'pose', 'autonomy', 'light', 'rgb', 'odom', 'drive', 'dynamics', 'scan', 'safety',
                        'safe_drive', 'imu'}
            if not required.issubset(state) or not actor:
                if time.monotonic()-began > limits['startup_wall_timeout_s']:
                    raise TimeoutError(f'startup data missing: {sorted(required-set(state))}, actor={len(actor)}')
                continue
            pose = state['pose']
            now = pose['time']
            first_pose_time = now if first_pose_time is None else first_pose_time
            light = state['light']
            if t_green is None and light.get('applied') == 'green':
                t_green = now
                report['green_observed_sim_s'] = now
            if case['schedule'] and t_green is not None and actor_future is None and (
                    last_command_s is None or now-last_command_s >= 1/actor_cfg['command_hz']):
                x, y, yaw = actor_pose_at(centerline, s0, case['schedule'], now-t_green, lateral)
                request = SetEntityPose.Request()
                request.entity.name = actor_cfg['model_name']
                request.entity.type = request.entity.MODEL
                request.pose.position.x, request.pose.position.y = x, y
                request.pose.position.z = actor_cfg['bottom_m']+actor_cfg['height_m']/2
                request.pose.orientation.z, request.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                actor_future = actor_client.call_async(request)
                commands.append({'requested_sim_s': now, 'elapsed_s': now-t_green, 'x_m': x, 'y_m': y,
                                 'requested_wall_s': time.monotonic()-began,
                                 'requested_wall_absolute_s': time.monotonic(), 'acknowledged': False})
                last_command_s = now
            if now-last_record < .10:
                if t_green is not None and now-t_green >= case['duration_s']:
                    break
                continue
            last_record = now
            position = {axis: float(pose['position'].get(axis, 0.)) for axis in ('x', 'y')}
            s, error = centerline.nearest_s(position['x'], position['y'])
            if last_s is not None:
                delta = (s-last_s+centerline.lap/2) % centerline.lap-centerline.lap/2
                progress += delta if abs(delta) <= 1 else 0.
            last_s = s
            footprint = rectangle(position['x'], position['y'], yaw_of(pose['orientation']), .20, .15)
            hit = any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))
            wall_collision += int(hit)
            autonomy = state['autonomy']
            signal_states.add(light['applied'])
            false_start |= early_start_detected(light['applied'], state['drive'].drive.speed, autonomy)
            false_start |= early_start_detected(light['applied'], state['safe_drive'].drive.speed, autonomy)
            latest_actor = actor[-1]
            gap = footprint.distance(rectangle(latest_actor[1], latest_actor[2], latest_actor[3],
                                               actor_cfg['length_m'], actor_cfg['width_m']))
            dynamics = state['dynamics']
            record = {
                't': now, 'x': position['x'], 'y': position['y'], 'yaw': yaw_of(pose['orientation']),
                's': s, 'progress_m': progress, 'centerline_error_m': error, 'wall_collision': hit,
                'elapsed_since_green_s': None if t_green is None else now-t_green, 'light': light['applied'],
                'ground_speed_mps': math.hypot(dynamics['truth_longitudinal_mps'], dynamics['truth_lateral_mps']),
                'truth_longitudinal_mps': dynamics['truth_longitudinal_mps'],
                'drive_speed_mps': state['drive'].drive.speed, 'safe_speed_mps': state['safe_drive'].drive.speed,
                'safety_reason': state['safety'].get('reason'), 'safety_clearance_m': state['safety'].get('swept_clearance_m'),
                'actor_x': latest_actor[1], 'actor_y': latest_actor[2], 'actor_stamp_s': latest_actor[0],
                'sampled_gap_m': gap,
                'autonomy': {k: autonomy.get(k) for k in (
                    'state', 'started', 'mode', 'behavior', 'reason', 'side', 'signal', 'marker_ids',
                    'speed_command_mps', 'steering_command_rad', 'swept_clearance_m', 'occupancy_padding_m',
                    'boundary_selection', 'boundary_from_memory', 'boundary_age_s', 'path_wall', 'wall_fallback',
                    'observed_path_ahead_m', 'fresh_boundary_accepted', 'target_speed_before_hold_mps',
                    'source_first_s', 'reference_stamp_s', 'sim_time_s', 'passing_enabled',
                    'boundary_candidate_reason', 'boundary_candidate_ahead_m', 'bilateral_alternative_confirmed',
                    'input_error', 'source_epoch')},
                'mcu_state': state.get('actuation', {}).get('state'),
            }
            if args.record_braking_diagnostics:
                record['braking_diagnostics'] = {
                    **dynamics,
                    'safe_command_stamp_s': stamp(state['safe_drive'].header.stamp),
                    'mcu_now_us': state.get('actuation', {}).get('now_us'),
                    'mcu_target_speed_mps': state.get('actuation', {}).get('target_speed_mps'),
                    'mcu_mean_wheel_mps': state.get('actuation', {}).get('mean_wheel_mps'),
                }
            trace.append(record)
            trace_stream.write(json.dumps(record, ensure_ascii=False)+'\n')
            if wall_collision > 3 or error > .35:
                outcome['abort'] = f'wall contact or route departure: samples={wall_collision}, error={error:.3f}'
                break
            if t_green is not None and now-t_green >= case['duration_s']:
                break
            if t_green is None and now-first_pose_time > limits['max_sim_after_green_margin_s']+30:
                raise TimeoutError('green not observed in time')
            if t_green is not None and now-t_green > case['duration_s']+limits['max_sim_after_green_margin_s']:
                raise TimeoutError('case sim window overrun')
        report['case_window'] = {'green_sim_s': t_green, 'end_sim_s': state['pose']['time'],
                                 'duration_s': case['duration_s']}
        # 일정이 끝난 뒤 actor는 마지막 명령 자세에 정지한다. 명령 응답을 먼저 회수한다.
        drain = time.monotonic()+8
        while actor_future is not None and time.monotonic() < drain:
            rclpy.spin_once(node, timeout_sec=.01)
            complete_actor_future()
        controller = 'local_pursuit'
        inputs = {topic for topic, _ in node.get_subscriber_names_and_types_by_node(controller, '/')}
        required_inputs = {'/scan', '/camera/color/image_raw', configured_feedback_topic(), '/imu/data'}
        report['autonomy_subscriptions'] = sorted(inputs)
        report['sensor_only_inputs'] = required_inputs.issubset(inputs) and inputs.issubset(required_inputs | {'/clock', '/parameter_events'})
        safety_inputs = {topic for topic, _ in node.get_subscriber_names_and_types_by_node('lidar_safety', '/')}
        safety_expected = {'/scan', '/imu/data', configured_feedback_topic(), '/drive'}
        report['safety_subscriptions'] = sorted(safety_inputs)
        report['safety_sensor_only_inputs'] = safety_expected.issubset(safety_inputs) and safety_inputs.issubset(safety_expected | {'/clock', '/parameter_events'})
        actor_topic = f"/model/{actor_cfg['model_name']}/pose"
        report['actor_pose_subscribers'] = sorted(i.node_name for i in node.get_subscriptions_info_by_topic(actor_topic))
        report['drive_publishers'] = sorted(i.node_name for i in node.get_publishers_info_by_topic('/drive'))
        report['safe_drive_publishers'] = sorted(i.node_name for i in node.get_publishers_info_by_topic('/drive/safe'))
        contact_info = subprocess.run(['gz', 'topic', '-i', '-t', CONTACT_TOPIC], cwd=REPO,
            capture_output=True, text=True, timeout=10)
        report['contact_topic_connection'] = {'topic': CONTACT_TOPIC, 'returncode': contact_info.returncode,
            'output': contact_info.stdout, 'stderr': contact_info.stderr,
            'reader_alive_at_graph_audit': readers[1].poll() is None}
        host_inputs = sorted({t for t, _ in node.get_subscriber_names_and_types_by_node('actuation_host', '/')})
        mcu_inputs = sorted({t for t, _ in node.get_subscriber_names_and_types_by_node('virtual_mcu', '/')})
        sink_inputs = sorted({t for t, _ in node.get_subscriber_names_and_types_by_node('guarded_sim_actuator', '/')})
        add_rate_metrics(report, [], scans, state.get('scan'), launch['lidar_rate_hz'])
        # 명시 STOP. 이미 고정 정지된 경우에도 실제 모델 정지와 출력 차단은 별도로 본다.
        if not mcu_stop_service.wait_for_service(timeout_sec=3):
            raise TimeoutError('MCU STOP service unavailable')
        # STOP 전 MCU 고정 판정의 기준: 요청 직전에 새로 받은 평가기 /clock 정수 ns. 아래
        # stop_start는 기존과 같이 요청 직후 수신 자세 시각이며 정지 대기 창에만 쓴다.
        stop_request_clock_ns, clock_refresh = ev.fresh_clock_ns(
            lambda: rclpy.spin_once(node, timeout_sec=.005), state, time.monotonic)
        future = mcu_stop_service.call_async(Trigger.Request())
        stop_start = state['pose']['time']
        report['stop_request_sim_s'] = stop_start
        report['stop_request_clock_ns'] = stop_request_clock_ns
        report['stop_request_clock_refresh'] = {**clock_refresh, 'pose_minus_clock_s':
            None if stop_request_clock_ns is None else stop_start-stop_request_clock_ns*1e-9}
        stop_origin = {axis: float(state['pose']['position'].get(axis, 0.)) for axis in ('x', 'y')}
        stable_since, stop_trace, last_stop = None, [], -math.inf
        deadline = time.monotonic()+limits['stop_wall_timeout_s']
        while state['pose']['time']-stop_start < acceptance['stop']['maximum_wait_sim_s']:
            if stop_requested or time.monotonic() > deadline or any(p.poll() is not None for p in processes):
                raise TimeoutError('stop check interrupted, timed out or process exited')
            rclpy.spin_once(node, timeout_sec=.01)
            now = state['pose']['time']
            if now > last_stop:
                p = {axis: float(state['pose']['position'].get(axis, 0.)) for axis in ('x', 'y')}
                yaw = yaw_of(state['pose']['orientation'])
                footprint = rectangle(p['x'], p['y'], yaw, .20, .15)
                hit = any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))
                stop_trace.append({'time': now, 'x': p['x'], 'y': p['y'], 'yaw': yaw, 'collision': hit,
                                   'ground_speed_mps': math.hypot(state['dynamics']['truth_longitudinal_mps'],
                                                                  state['dynamics']['truth_lateral_mps']),
                                   'dynamics_source_s': state['dynamics'].get('sim_time_s'),
                                   'odom_source_s': stamp(state['odom'].header.stamp),
                                   'safe_command_source_s': stamp(state['safe_drive'].header.stamp),
                                   'mcu_source_us': state.get('actuation', {}).get('now_us')})
                last_stop = now
            if (abs(state['odom'].twist.twist.linear.x) < .01 and abs(state['dynamics']['truth_longitudinal_mps']) < .01
                    and abs(state['dynamics']['truth_lateral_mps']) < .01):
                stable_since = now if stable_since is None else stable_since
                if now-stable_since >= acceptance['stop']['stable_hold_sim_s'] and future.done() and future.result().success:
                    # 비동기 진단의 저속 두 표본만으로 실제 정지 완료를 선언하지 않는다.
                    # 평가용 밀집 자세 전 구간도 기존 속도/시간 한도를 만족해야 한다.
                    evidence = ev.stop_pose_evidence(tuple(ego), now, acceptance['stop']['stable_hold_sim_s'])
                    if evidence['passed']:
                        break
            else:
                stable_since = None
        stop_end = stop_trace[-1]['time'] if stop_trace else stop_start
        evidence = ev.stop_pose_evidence(tuple(ego), stop_end, acceptance['stop']['stable_hold_sim_s'])
        report['stop'] = {
            'stable': (stable_since is not None and stop_end-stable_since >= acceptance['stop']['stable_hold_sim_s']
                       and future.done() and future.result().success and evidence['passed']),
            'pose_evidence': evidence,
            'wait_sim_s': stop_end-stop_start,
            'displacement_m': math.dist([stop_origin['x'], stop_origin['y']],
                                        [stop_trace[-1][a] for a in ('x', 'y')]) if stop_trace else 0.,
            'wall_collision_samples': sum(r['collision'] for r in stop_trace),
        }
        (output/'stop_trace.json').write_text(json.dumps(stop_trace, indent=2), encoding='utf-8')
        final = state.get('actuation', {})
        latch_timing = ev.stop_latch_timing(actuation_trace, stop_request_clock_ns,
                                            acceptance['stop']['mcu_state'], acceptance['stop']['mcu_reason'])
        report['actuation'] = {
            'arm_acknowledged': bool(arm_future is not None and arm_future.done() and arm_future.result().success),
            'final_mcu': final, 'final_output': state.get('actuation_output'),
            'positive_command_samples': sum(s['target_speed_mps'] > 0 for s in actuation_trace),
            'latched_before_stop_samples': latch_timing['latched_before_stop_samples'],
            'stop_latch_timing': latch_timing,
            'host_subscriptions': host_inputs, 'mcu_subscriptions': mcu_inputs, 'sink_subscriptions': sink_inputs,
        }
        expected = [({'/drive/safe', '/actuation/rx'}, host_inputs),
                    ({configured_feedback_topic(), '/actuation/tx', '/actuation/output_status'}, mcu_inputs),
                    ({'/actuation/status'}, sink_inputs)]
        report['actuation']['sensor_only_inputs'] = all(e.issubset(i) and set(i).issubset(e | {'/clock', '/parameter_events'})
                                                       for e, i in expected)
        report['actuation_stop_passed'] = (report['actuation']['arm_acknowledged'] and
            final.get('reason') == acceptance['stop']['mcu_reason'] and final.get('state') == acceptance['stop']['mcu_state']
            and (state.get('actuation_output') or {}).get('speed_mps') == 0.)
        report['signal_states'] = sorted(signal_states)
        report['false_start'] = false_start
        report['wall_collision_samples'] = wall_collision
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        print('G1-b run failed:', report['error'], flush=True)
    finally:
        if state.get('actuation', {}).get('reason') == 'control_deadline_missed':
            deadline = time.monotonic()+1.
            while not actuation_diagnostics and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
        # ROS 콜백도 이 시점부터 spin하지 않는다. 정상 STOP 검사 또는 실패 시점까지
        # 이미 받은 원시 자료를 모두 남기되, 저장/서버 종료 중 뒤늦은 tail은 받지 않는다.
        capture_gate.close()
        report['observation_closed_before_serialization'] = True
        report['observation_last_ego_sim_s'] = ego[-1][0] if ego else None
        report['abort'] = outcome['abort']
        if args.record_wheel_contacts:
            path = output/'wheel_contacts.jsonl'
            path.write_text(''.join(json.dumps(row)+'\n' for row in wheel_contacts), encoding='utf-8')
            report['wheel_contacts'] = {
                'samples': len(wheel_contacts), 'sha256': sha256(path),
                'scope': 'evaluation contact events only; silence is not proof of zero load; no control input',
            }
        (output/'actuation_diagnostics.json').write_text(json.dumps(actuation_diagnostics, indent=2), encoding='utf-8')
        (output/'actuation_trace.json').write_text(json.dumps(actuation_trace), encoding='utf-8')
        (output/'acquisition.json').write_text(json.dumps(acquisition_records), encoding='utf-8')
        if args.record_braking_diagnostics:
            path = output/'braking_diagnostics.jsonl'
            path.write_text(''.join(json.dumps(row)+'\n' for row in braking_diagnostics), encoding='utf-8')
            report['braking_diagnostics'] = {
                'samples': len(braking_diagnostics), 'sha256': sha256(path),
                'scope': 'evaluation only; original drivetrain source timestamps retained; no control or verdict change',
            }
        report['actuation_diagnostics'] = {'samples': len(actuation_diagnostics),
            'control_deadline_samples': sum(r.get('type') == 'control_deadline_diagnostic' for r in actuation_diagnostics),
            'scope': 'pre-shutdown fault diagnostics; no message alone does not prove no delay'}
        report['sequential_acquisition_verified'] = bool(acquisition_records) and all(
            r['max_capture_time_error_s'] <= .0021001 and r['discarded_source_gaps'] == 0 for r in acquisition_records)
        report['actor_frames_seen'] = sorted(map(list, actor_frames))[:10]
        report['actor_pose_rejected_nonincreasing_samples'] = rejected_actor_times
        report['ego_pose_rejected_nonincreasing_samples'] = rejected_ego_times
        report['evaluation_readers'] = {'streams': reader_audit,
            'valid': reader_audit['ego']['handled_messages'] > 0 and all(
                a['parse_errors'] == 0 and a['handler_errors'] == 0 for a in reader_audit.values()),
            'scope': 'reader parse/handler success; contact event-only silence is not no-contact coverage'}
        probe = json.loads(CONTACT_PROBE_REPORT.read_text(encoding='utf-8')) if CONTACT_PROBE_REPORT.exists() else {}
        connection = report.get('contact_topic_connection', {})
        report['contact_evidence'] = {'validated_source': bool(
            probe.get('interpretation', {}).get('moved_contact_reported') is True
            and probe.get('server_returncode') == 0 and ev.contact_connection_valid(connection)
            and reader_audit['contact']['parse_errors'] == 0 and reader_audit['contact']['handler_errors'] == 0),
            'known_contact_probe': str(CONTACT_PROBE_REPORT.relative_to(REPO)),
            'known_contact_probe_sha256': sha256(CONTACT_PROBE_REPORT) if CONTACT_PROBE_REPORT.exists() else None,
            'topic_connection': connection, 'observed_events': len(contacts),
            'scope': 'known-contact source capability + this-run connection/error checks; not continuous physical no-contact proof'}
        node.destroy_node()
        rclpy.shutdown()
        report['gazebo_stop_service'] = stop_gazebo_service(processes[0] if processes else None,
                                                            log_path.read_text(errors='replace'), os.environ['GZ_PARTITION'])
        forced = []
        for process in readers+processes[1:]:
            forced += stop_owned(process)
        cleanup = stop_launch(processes[0] if processes else None)
        log.close()
        reader_log.close()
        trace_stream.close()
        scan_stream.close()
        status_stream.close()
        for name in ('scans_received', 'autonomy_status'):
            report[f'{name}_sha256'] = sha256(output/f'{name}.jsonl')
        report['process_exit_codes'] = [p.poll() for p in processes+readers]
        report['shutdown_audit'] = audit_shutdown(log_path.read_text(errors='replace'),
                                                  processes[0].poll() if processes else None, cleanup or None)
        report['remaining_test_pids'] = sorted({pid for p in processes+readers for pid in group_processes(p.pid)})
        report['reader_forced_cleanup'] = forced
        report['shutdown_clean'] = (report['shutdown_audit']['clean'] and not report['remaining_test_pids'] and not forced
                                    and all(p.poll() in (0, -signal.SIGINT, 130) for p in processes[1:]+readers)
                                    and report['gazebo_stop_service']['verified'])
        report['source_unchanged'] = all(sha256(REPO/rel) == digest for rel, digest in sources.items())
        if args.record_follow_decisions:
            path = output/'follow_decisions.jsonl'
            report['follow_decisions'] = {
                'exists': path.is_file(), 'sha256': sha256(path) if path.is_file() else None,
                'scope': 'decision-stage snapshots only; completeness and replay are separate checks',
            }
        report['wall_elapsed_s'] = time.monotonic()-began
        for name, rows in (('ego_poses', ego), ('actor_poses', actor)):
            path = output/f'{name}.jsonl'
            path.write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
            report[f'{name}_sha256'] = sha256(path)
            report[f'{name}_samples'] = len(rows)
        path = output/'actor_contacts.jsonl'
        path.write_text(''.join(json.dumps(r)+'\n' for r in contacts), encoding='utf-8')
        report['actor_contacts_sha256'] = sha256(path)
        (output/'actor_commands.json').write_text(json.dumps(commands, indent=1), encoding='utf-8')
        (output/'trace.json').write_text(json.dumps(trace, ensure_ascii=False), encoding='utf-8')
        try:
            report['evaluation'] = ev.evaluate_run(
                config=config, case=case, mode=args.mode, t_green=t_green, trace=trace, ego=ego, actor=actor,
                contacts=contacts, commands=commands, stop_start=report.get('stop_request_sim_s'),
                schedule_pose=schedule_pose, centerline=centerline, s0=s0,
                road=road_polygon(output/'runtime_world.sdf') if (output/'runtime_world.sdf').exists() else None,
                runtime_world=(output/'runtime_world.sdf').read_text(encoding='utf-8') if (output/'runtime_world.sdf').exists() else '',
                contact_evidence=report['contact_evidence'])
        except Exception as error:
            report['evaluation'] = {'error': f'{type(error).__name__}: {error}'}
        ev.finalize_verdict(report, args.mode)
        if runtime_lidar_profile:
            try:
                with (output/'scans_received.jsonl').open(encoding='utf-8') as stream:
                    received_scans = [json.loads(line) for line in stream if line.strip()]
                stress = audit_injection(acquisition_records, received_scans, lidar_requested)
                stress['runtime_profile_unchanged'] = (
                    sha256(runtime_lidar_profile) == report['lidar_test_profile']['runtime_sha256'])
                stress['verified'] = stress['verified'] and stress['runtime_profile_unchanged']
                stress['input_sha256'] = {name: sha256(output/name) for name in
                    ('acquisition.json', 'scans_received.jsonl', 'lidar_test_profile.json')}
            except Exception as error:
                stress = {'verified': False, 'error': f'{type(error).__name__}: {error}'}
            require_injection_evidence(report, stress)
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({k: report.get(k) for k in ('case', 'seed', 'mode', 'passed', 'harness_valid', 'error', 'abort')}
                         | {'gates': report.get('gates')}, ensure_ascii=False, indent=1), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
