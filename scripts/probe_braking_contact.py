#!/usr/bin/env python3
"""고정 /drive 명령의 평가 전용 제동 대조. 보호층과 가상 MCU는 유지한다.

official은 현행 파생본, flat은 충돌 노면만 높이 3 mm 단일 판으로 바꾼
분리 시험이다. 원본 파일/차량/센서/모터 물성은 바꾸지 않는다.
차체 정답은 결과 평가·시험 중단에만 사용하고 목표 속도에는 쓰지 않는다.
"""
import argparse
from array import array
from datetime import datetime, timezone
import gc
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zipfile

from braking_command_profile import load_profile, audit_requests
from evaluation_gc import EvaluationGC
from evaluation_startup import PausedStartup, STATS_TOPIC, startup_arguments, audit_startup
from arena_vehicle_interface.evaluation_schedule import TOPIC as SCHEDULED_TOPIC, EVENT_TOPIC, audit_scheduled
from scheduled_command import (GridSchedule, ProfileSampler, REGISTERED_PERIOD_NS, ScheduleFaultInjection,
                               align_up, audit_all_ticks, audit_scheduled_requests)
from validate_g1_traffic import REPO, SOURCE_FILES, json_reader, sha256, stop_owned
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch

PLUGIN_BINARY = 'install/arena_gazebo/lib/libarena-single-motor-system.so'
COMMAND_TRACE_TOPIC = '/model/arena_car/eval/command_trace'
# 격자 모드 평가기 시간 계측의 원시 배열 순서(int64 little-endian, 길이는 report에 기록).
TIMING_ARRAYS = ('poll_wall_before_spin_ns', 'poll_wall_after_spin_ns', 'poll_node_clock_ns',
                 'clock_msg_sim_ns', 'clock_msg_wall_ns')


def request_speed(elapsed):
    """0→1 m/s 1초, 유지 1초, 0까지 0.5초, 이후 정지. 시간만 입력."""
    if elapsed < 0:
        return 0.
    if elapsed < 1:
        return elapsed
    if elapsed < 2:
        return 1.
    return max(0., 1.-2.*(elapsed-2.))


def flat_world(source, destination):
    root = ET.parse(source)
    world = root.getroot().find('world')
    count = 0
    for link in world.findall('model/link'):
        for collision in list(link.findall('collision')):
            # 바닥/도로/잔디만 제거한다. 벽·마커·차량 충돌체는 유지.
            name = collision.get('name', '')
            if name == 'ground_plane_link__ground_collision' or name.startswith(('surface_', 'grass_')):
                link.remove(collision)
                count += 1
    if count < 2:
        raise ValueError('시험 노면 충돌체 이름이 변경됐습니다.')
    model = ET.SubElement(world, 'model', name='eval_flat_support')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='floor')
    collision = ET.SubElement(link, 'collision', name='collision')
    ET.SubElement(collision, 'pose').text = '0 0 -0.047 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = '40 40 0.1'
    # 공식 도로와 같이 surface/friction을 지정하지 않는다.
    for tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map', 'metalness_map'):
        for node in world.findall(f'.//{tag}'):
            if node.text and '://' not in node.text and not Path(node.text).is_absolute():
                node.text = str((source.parent/node.text).resolve())
    root.write(destination, encoding='utf-8', xml_declaration=True)
    return count


def evaluation_extra(physics_step=None, world=None, slot=None):
    """기본 실행 인자는 이전과 같다. 평가 step은 명시했을 때만 launch에 전달한다."""
    extra = [f'evaluation_physics_step:={physics_step}'] if physics_step else []
    if world is not None:
        extra += [f'world_override:={world}', f"test_spawn_x:={slot['x']}",
                  f"test_spawn_y:={slot['y']}", f"test_spawn_yaw:={slot['yaw_rad']}"]
    return extra


def schedule_arguments(schedule, phase_ms, profile_given):
    """선택형 격자 발행 인자 검사. 기본 poll이면 None(기존 발행 경로)."""
    if schedule == 'poll':
        if phase_ms is not None:
            raise ValueError('--schedule-phase-ms requires --command-schedule sim_grid')
        return None
    if schedule != 'sim_grid':
        raise ValueError('unknown command schedule')
    if not profile_given:
        raise ValueError('sim_grid schedule requires --command-profile')
    if phase_ms not in ('0', '10'):
        raise ValueError('sim_grid schedule requires registered --schedule-phase-ms 0 or 10')
    return {'mode': 'sim_grid', 'period_ns': REGISTERED_PERIOD_NS, 'phase_ns': int(phase_ms)*1_000_000}


def fault_arguments(kind, at_ms, delay_ms, grid_spec, sampler, output_name):
    """평가 전용 발행 지연 주입 인자 검사. 기본(None)이면 기존 경로 그대로."""
    if kind is None:
        if at_ms is not None or delay_ms is not None:
            raise ValueError('--evaluation-schedule-fault-at-ms/-delay-ms require --evaluation-schedule-fault')
        return None
    if not grid_spec:
        raise ValueError('schedule fault injection requires --command-schedule sim_grid')
    if at_ms is None or delay_ms is None:
        raise ValueError('schedule fault injection requires explicit target and delay')
    if 'inject' not in output_name:
        raise ValueError('fault injection output folder name must contain "inject"')
    rel_ns, delay_ns = ScheduleFaultInjection.validate(kind, at_ms, delay_ms, sampler, grid_spec['period_ns'])
    return {'kind': kind, 'target_rel_ns': rel_ns, 'delay_ns': delay_ns}


def trace_arguments(enabled, grid_spec):
    """평가 전용 플러그인 명령 전환 이벤트. 기본 꺼짐이며 격자 발행 실행에서만 허용한다."""
    if not enabled:
        return []
    if not grid_spec:
        raise ValueError('--plugin-command-trace requires --command-schedule sim_grid')
    return ['evaluation_command_trace:=true']


def command_trace_audit(events, reader_audit):
    """플러그인 이벤트 수신 완결성. 순번 공백(유실)·첫 갱신·요약·적용 이벤트가 있어야 유효."""
    seqs = [e.get('event_seq') for e in events]
    kinds = {}
    for event in events:
        kinds[event.get('type')] = kinds.get(event.get('type'), 0)+1
    applies = [e for e in events if e.get('type') == 'apply']
    checks = {
        'reader_clean': reader_audit['handled_messages'] > 0 and not reader_audit['parse_errors']
                        and not reader_audit['handler_errors'],
        'sequence_contiguous_from_1': bool(seqs) and seqs == list(range(1, len(seqs)+1)),
        'first_update_present': kinds.get('first_update', 0) == 1,
        'summary_present': kinds.get('summary', 0) >= 1,
        'speed_and_steering_applies': any(e.get('channel') == 'speed' for e in applies)
                                      and any(e.get('channel') == 'steering' for e in applies),
    }
    last_summary = next((e for e in reversed(events) if e.get('type') == 'summary'), None)
    return {'checks': checks, 'valid': all(checks.values()), 'events': len(events), 'types': kinds,
            'last_summary': last_summary, 'reader_audit': dict(reader_audit)}


def clock_summary(values, begin_ns, end_ns):
    """평가기 구독으로 받은 /clock 값의 간격 분포. 발행 해상도와 수신 유실을 구분하지 못한다."""
    inside = [v for v in values if begin_ns <= v <= end_ns]
    deltas = [b-a for a, b in zip(inside, inside[1:])]
    histogram = {}
    for delta in deltas:
        histogram[delta] = histogram.get(delta, 0)+1
    return {'messages': len(inside), 'regressions': sum(d < 0 for d in deltas), 'repeats': sum(d == 0 for d in deltas),
            'min_positive_delta_ns': min((d for d in deltas if d > 0), default=None),
            'max_delta_ns': max(deltas, default=None),
            'delta_ns_top': sorted(histogram.items(), key=lambda kv: -kv[1])[:10],
            'scope': 'best-effort 구독 기준. 간격이 step보다 크면 발행 묶음 또는 수신 유실이며 둘을 구분하지 않는다.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--surface', choices=('official', 'flat'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--zero-speed-mode', default='legacy',
                        choices=('legacy', 'reset_integral', 'no_lag', 'passive_damping'))
    parser.add_argument('--command-profile', type=Path,
                        help='선택형 사전 고정 속도/조향 요청 JSON. 생략하면 기존 완만한 제동 시험.')
    parser.add_argument('--physics-step', choices=('0.0005', '0.00025'),
                        help='평가 전용 물리 step 민감도. 생략하면 월드 기본 1 ms.')
    parser.add_argument('--command-schedule', choices=('poll', 'sim_grid'), default='poll',
                        help='poll=기존 now-last>=.019 발행(기본). sim_grid=시뮬레이션 시계 20 ms 정수 격자 발행(평가 전용)')
    parser.add_argument('--schedule-phase-ms', choices=('0', '10'),
                        help='sim_grid 격자의 sim 시각 위상(20 ms 주기 기준 0 또는 10 ms)')
    parser.add_argument('--evaluation-schedule-fault', choices=('late', 'miss'),
                        help='평가 전용 발행 지연 주입(기본 꺼짐). sim_grid 전용, 출력 폴더 이름에 inject 필요')
    parser.add_argument('--evaluation-schedule-fault-at-ms', type=int,
                        help='주입 목표 tick의 프로필 기준 시각(ms, 등록 격자·유지값 tick)')
    parser.add_argument('--evaluation-schedule-fault-delay-ms', type=int,
                        help='late: 등록 한도 초과~한 주기 미만, miss: 한 주기 이상~두 주기 미만')
    parser.add_argument('--plugin-command-trace', action='store_true',
                        help='평가 전용 플러그인 명령 수신/적용 전환 이벤트(기본 꺼짐, sim_grid 전용)')
    parser.add_argument('--evaluation-gc', choices=('native', 'defer'), default='native',
                        help='평가기 순환 GC: native=원래 동작, defer=짧은 sim_grid 시험 동안만 연기')
    parser.add_argument('--evaluation-startup', choices=('service', 'embedded_paused'), default='service',
                        help='평가 전용 초기 차량 포함·paused 시작/연결 준비 확인. 기본은 기존 서비스 스폰')
    parser.add_argument('--evaluation-scheduled-apply',action='store_true',
                        help='평가 전용 보호 출력의 20 ms 미래 적용/ACK. embedded_paused 전용')
    args = parser.parse_args()
    grid_spec = schedule_arguments(args.command_schedule, args.schedule_phase_ms, args.command_profile is not None)
    profile = load_profile(args.command_profile) if args.command_profile else None
    sampler = ProfileSampler(profile) if grid_spec else None
    fault_spec = fault_arguments(args.evaluation_schedule_fault, args.evaluation_schedule_fault_at_ms,
                                 args.evaluation_schedule_fault_delay_ms, grid_spec, sampler, args.output.name)
    trace_extra = trace_arguments(args.plugin_command_trace, grid_spec)
    startup_extra = startup_arguments(args.evaluation_startup, bool(grid_spec), bool(trace_extra))
    if args.evaluation_scheduled_apply:
        if args.evaluation_startup!='embedded_paused' or not trace_extra or not grid_spec:
            raise ValueError('scheduled apply requires embedded_paused/grid/trace')
        startup_extra += ['evaluation_scheduled_apply:=true']
    if startup_extra and args.physics_step:
        raise ValueError('initial startup validation is registered for default 1 ms only')
    gc_policy = EvaluationGC(args.evaluation_gc, grid=bool(grid_spec))
    output = args.output.resolve()
    if not output.is_relative_to(REPO/'artifacts'):
        raise ValueError('시험 자료는 artifacts 안에 보존합니다.')
    output.mkdir(parents=True, exist_ok=False)
    source_paths = [*SOURCE_FILES, str(Path(__file__).relative_to(REPO))]
    source_paths += ['scripts/braking_command_profile.py', 'scripts/evaluation_gc.py', 'scripts/evaluation_startup.py']
    source_paths += ['src/arena_gazebo/include/arena_gazebo/scheduled_apply.hpp',
                     'src/arena_vehicle_interface/arena_vehicle_interface/evaluation_schedule.py',
                     'src/arena_vehicle_interface/arena_vehicle_interface/scheduled_actuator.py',
                     'src/arena_vehicle_interface/setup.py']
    if grid_spec:
        source_paths += ['scripts/scheduled_command.py', 'src/arena_gazebo/include/arena_gazebo/command_trace.hpp']
    if args.command_profile:
        source_paths.append(args.command_profile.resolve().relative_to(REPO).as_posix())
    sources = {rel: sha256(REPO/rel) for rel in source_paths}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as bundle:
        for rel in sources:
            bundle.write(REPO/rel, rel)
    world_dir = REPO/'src/arena_gazebo/worlds/it_arena_official'
    slot = next(s for s in json.loads((world_dir/'scene.json').read_text())['starting_grid']['slots'] if s['index'] == 3)
    report = {'started_at_utc': datetime.now(timezone.utc).isoformat(), 'surface': args.surface,
              'source_sha256': sources, 'scope': 'fixed request, same LiDAR safety and virtual MCU; not autonomous driving',
              'input_valid': False, 'stopped': False, 'shutdown_clean': False}
    report['spawn'] = {key: slot[key] for key in ('x', 'y', 'yaw_rad')}
    report['zero_speed_mode'] = args.zero_speed_mode
    report['evaluation_scheduled_apply_requested']=args.evaluation_scheduled_apply
    report['physics_step_override_s'] = args.physics_step
    if grid_spec:
        report['command_schedule'] = dict(grid_spec)
        # 실제 로드될 플러그인 바이너리(설치 symlink 대상) 해시. 소스 해시와 별도 증거.
        report['plugin_binary_sha256'] = sha256(REPO/PLUGIN_BINARY)
        report['plugin_command_trace_requested'] = bool(trace_extra)
        report['evaluation_fault_injection_requested'] = fault_spec
    report['command_profile'] =({'path': args.command_profile.resolve().relative_to(REPO).as_posix(),
                                  'sha256': sha256(args.command_profile), 'duration_s': profile.duration_s,
                                  'brake_window_s': profile.brake_window_s, 'source': profile.source}
                                 if profile else None)
    stop_elapsed = profile.duration_s+1. if profile else 4.5
    finish_elapsed = stop_elapsed+1.
    if args.surface == 'flat':
        world = output/'flat_world.sdf'
        report['replaced_ground_collisions'] = flat_world(world_dir/'world.sdf', world)
    extra = evaluation_extra(args.physics_step, world if args.surface == 'flat' else None, slot)+trace_extra+startup_extra
    partition = f'arena_autonomy_test_{os.getpid()}'
    os.environ.update(ROS_DOMAIN_ID=str(150+os.getpid()%30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', GZ_PARTITION=partition)
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fixed_braking_evaluator',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    state, trace, contacts, feedback = {}, [], [], []
    requests, safe_commands = [], []
    audit = dict(json_messages=0, handled_messages=0, parse_errors=0, handler_errors=0, last_error=None)
    # sim_grid 전용 전달 추적. 기존 폴링 경로에는 구독을 추가하지 않는다.
    mcu_statuses, output_statuses, clock_values = [], [], []
    scheduled_sent, scheduled_events, scheduled_states = [], [], []
    schedule = start_ns = injection = None
    last_clock_msg = [None]
    # sim_grid 전용 자연 지연 계측(벽시계 loop/콜백/GC, /clock 수신 벽시계). 판정이나 발행 값에는 쓰지 않는다.
    timing = {name: array('q') for name in TIMING_ARRAYS}
    callback_stats, slow_callbacks, gc_events = {}, [], []
    plugin_events, trace_audit = [], dict(json_messages=0, handled_messages=0, parse_errors=0, handler_errors=0,
                                          last_error=None)

    def timed(name, function):
        """콜백 벽시계 소요를 집계한다. 기본 폴링 경로에는 감싸지 않는다."""
        if not grid_spec:
            return function
        stats = callback_stats.setdefault(name, {'count': 0, 'total_ns': 0, 'max_ns': 0})

        def wrapper(message):
            begin = time.monotonic_ns()
            try:
                return function(message)
            finally:
                spent = time.monotonic_ns()-begin
                stats['count'] += 1
                stats['total_ns'] += spent
                stats['max_ns'] = max(stats['max_ns'], spent)
                if spent >= 2_000_000 and len(slow_callbacks) < 5000:
                    slow_callbacks.append({'callback': name, 'wall_begin_ns': begin, 'duration_ns': spent})
        return wrapper

    def on_gc(phase, info):
        if len(gc_events) < 20000:
            gc_events.append({'phase': phase, 'generation': info.get('generation'), 'wall_ns': time.monotonic_ns(),
                              'collected': info.get('collected')})
    if grid_spec:
        gc.callbacks.append(on_gc)
    interrupted = False
    def interrupt(*_):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    def dynamics(message):
        row = json.loads(message.data)
        state['dynamics'] = row
        trace.append({**row, 'mcu': state.get('mcu'), 'safe': state.get('safe'), 'request': state.get('request'),
                      'safety': state.get('safety')})
    def safe_command(message):
        row = {'sim_s': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
               'speed_mps': message.drive.speed, 'steering_rad': message.drive.steering_angle}
        safe_commands.append(row)
        state.update(safe=row['speed_mps'], safe_command=row)
    subscriptions = [
        node.create_subscription(String, '/sim/drivetrain', timed('drivetrain', dynamics), 100),
        node.create_subscription(String, '/actuation/status',
                                 timed('mcu_state', lambda m: state.update(mcu=json.loads(m.data))), 100),
        node.create_subscription(String, '/safety/status',
                                 timed('safety_state', lambda m: state.update(safety=json.loads(m.data))), 100),
        node.create_subscription(AckermannDriveStamped, '/drive/safe', timed('safe', safe_command), 100),
        node.create_subscription(JointState, '/drive_motor/encoder',
                                 timed('encoder', lambda m: feedback.append(list(m.name))), qos_profile_sensor_data),
    ]
    if grid_spec:
        def on_clock(message):
            value = message.clock.sec*1_000_000_000+message.clock.nanosec
            last_clock_msg[0] = value
            if len(clock_values) < 400_000:
                clock_values.append(value)
                timing['clock_msg_sim_ns'].append(value)
                timing['clock_msg_wall_ns'].append(time.monotonic_ns())
        def on_mcu_status(message):
            data = json.loads(message.data)
            mcu_statuses.append({k: data.get(k) for k in ('now_us', 'target_speed_mps', 'steering_rad', 'state',
                                                          'reason', 'accepted_seq', 'deadline_us')})
        def on_output_status(message):
            data = json.loads(message.data)
            output_statuses.append({k: data.get(k) for k in ('now_us', 'speed_mps', 'blocked', 'reason', 'last_status_us')})
        subscriptions += [
            node.create_subscription(Clock, '/clock', timed('clock_own', on_clock),
                                     QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)),
            node.create_subscription(String, '/actuation/status', timed('mcu_trace', on_mcu_status), 1000),
            node.create_subscription(String, '/actuation/output_status', timed('output_trace', on_output_status), 1000),
        ]
    arm_client = node.create_client(Trigger, '/actuation/arm')
    if args.evaluation_scheduled_apply:
        subscriptions += [
            node.create_subscription(String,SCHEDULED_TOPIC,lambda m:scheduled_sent.append({'packet':m.data}),1000),
            node.create_subscription(String,EVENT_TOPIC,lambda m:scheduled_events.append(json.loads(m.data)),1000),
            node.create_subscription(String,'/actuation/scheduled_status',lambda m:scheduled_states.append(json.loads(m.data)),1000),
        ]
    stop_client = node.create_client(Trigger, '/actuation/stop')
    log_path = output/'simulation.log'
    log = log_path.open('w')
    reader_log = (output/'readers.log').open('w')
    process = reader = trace_reader = stats_reader = arm = stop_future = None
    startup = PausedStartup(partition,scheduled=args.evaluation_scheduled_apply) if startup_extra else None
    startup_audit = dict(json_messages=0, handled_messages=0, parse_errors=0, handler_errors=0, last_error=None)
    report['evaluation_startup'] = startup.record if startup else {'mode': 'service'}
    start = last_command = last_print = None
    began = time.monotonic()

    def on_trace(data):
        event = json.loads(data['data'])
        if type(event) is not dict:
            raise ValueError('trace event must be an object')
        plugin_events.append(event)
    try:
        gc_policy.start()
        process = subprocess.Popen(['ros2', 'launch', '--noninteractive', 'arena_bringup', 'simulation.launch.py',
            'headless:=true', 'autonomy:=false', 'autonomy_mode:=local_pursuit', 'track:=official', 'grid_slot:=3',
            'actuation_mode:=virtual_mcu', 'lidar_acquisition:=sequential', 'lidar_rate_hz:=10',
            'lidar_compensation:=both', 'depth_camera:=false', 'speed_bump:=false', 'traffic_light:=false',
            'render_backend:=wsl_nvidia', 'batch_static_visuals:=true', 'sensor_wall_timeout_s:=30',
            'evaluation_wheel_contacts:=true', f'evaluation_zero_speed_mode:={args.zero_speed_mode}',
            f'runtime_artifacts:={output}', *extra], cwd=REPO,
            stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        reader = json_reader(['gz', 'topic', '-e', '-t', '/eval/wheels/contact', '--json-output'],
                             contacts.append, reader_log, audit)
        if trace_extra:
            trace_reader = json_reader(['gz', 'topic', '-e', '-t', COMMAND_TRACE_TOPIC, '--json-output'],
                                       on_trace, reader_log, trace_audit)
        if startup:
            stats_reader = json_reader(['gz', 'topic', '-e', '-t', STATS_TOPIC, '--json-output'],
                                       startup.observe, reader_log, startup_audit)
        while True:
            gc_policy.check()
            wall_before_spin = time.monotonic_ns()
            rclpy.spin_once(node, timeout_sec=.005)
            if interrupted or time.monotonic()-began > 600:
                raise TimeoutError('interrupted or wall timeout')
            if process.poll() is not None or reader.poll() is not None or (
                    trace_reader is not None and trace_reader.poll() is not None) or (
                    stats_reader is not None and stats_reader.poll() is not None):
                raise RuntimeError('owned process exited before completion')
            if startup and not startup.poll(node, arm_client, stop_client,
                                            node.get_clock().now().nanoseconds, plugin_events):
                continue
            mcu = state.get('mcu', {})
            data = state.get('dynamics')
            if not data or not mcu:
                continue
            now = node.get_clock().now().nanoseconds*1e-9
            if mcu['state'] in ('ESTOP_LATCHED', 'FAULT_LATCHED') and stop_future is None:
                raise RuntimeError('unexpected MCU latch: '+mcu['reason'])
            if arm is None and now > 1 and mcu['stationary_ready'] and mcu['output_ready'] and arm_client.service_is_ready():
                arm = arm_client.call_async(Trigger.Request())
            if arm is not None and arm.done() and start is None:
                if not arm.result().success:
                    raise RuntimeError('ARM refused')
                if profile:
                    report['drive_publishers'] = sorted(i.node_name for i in node.get_publishers_info_by_topic('/drive'))
                    report['safe_drive_publishers'] = sorted(i.node_name for i in node.get_publishers_info_by_topic('/drive/safe'))
                    report['drive_graph_valid'] = (report['drive_publishers'] == ['fixed_braking_evaluator']
                        and report['safe_drive_publishers'] == ['lidar_safety'])
                    if not report['drive_graph_valid']:
                        raise RuntimeError('unexpected command publisher graph')
                if grid_spec:
                    # ARM 확인 시계값 이후 첫 격자점에서 발행을 시작하고, 1 s 뒤 첫 격자점을 프로필 0으로 둔다.
                    now_ns = node.get_clock().now().nanoseconds
                    origin_ns = align_up(now_ns, grid_spec['period_ns'], grid_spec['phase_ns'])
                    start_ns = align_up(now_ns+1_000_000_000, grid_spec['period_ns'], grid_spec['phase_ns'])
                    schedule = GridSchedule(grid_spec['period_ns'], grid_spec['phase_ns'], origin_ns)
                    report['command_schedule'].update(arm_observed_ns=now_ns, origin_ns=origin_ns, start_ns=start_ns)
                    if fault_spec:
                        injection = ScheduleFaultInjection(fault_spec['kind'], start_ns+fault_spec['target_rel_ns'],
                                                           fault_spec['delay_ns'])
                    start = start_ns/1e9
                else:
                    start = now+1.
                report['command_start_sim_s'] = start
                # 렌더링/물리 실시간 비율은 시뮬레이션 시간과 구분해 기록한다.
                report['arm_done_wall_s'] = time.monotonic()-began
            elapsed = now-start if start is not None else -1.
            if grid_spec:
                # ARM 전에는 발행하지 않는다. 한 번 읽은 시계값으로 판정과 header stamp를 같이 쓴다.
                if schedule is not None:
                    clock_now = node.get_clock().now()
                    now_ns = clock_now.nanoseconds
                    elapsed = (now_ns-start_ns)/1e9
                    timing['poll_wall_before_spin_ns'].append(wall_before_spin)
                    timing['poll_wall_after_spin_ns'].append(time.monotonic_ns())
                    timing['poll_node_clock_ns'].append(now_ns)
                    # 평가 전용 주입이 있을 때만 목표 tick 주변 poll을 보류한다(기본 None).
                    tick = None if injection is not None and injection.withhold(now_ns) else schedule.poll(now_ns)
                    if tick is not None:
                        command = AckermannDriveStamped()
                        command.header.stamp = clock_now.to_msg()
                        command.header.frame_id = 'base_link'
                        speed, steering = sampler.at_ns(tick.scheduled_ns-start_ns)
                        command.drive.speed, command.drive.steering_angle = speed, steering
                        publisher.publish(command)
                        state['request'] = {'sim_s': now_ns/1e9, 'speed_mps': speed, 'steering_rad': steering,
                                            'scheduled_ns': tick.scheduled_ns, 'observed_ns': tick.observed_ns,
                                            'header_ns': command.header.stamp.sec*1_000_000_000+command.header.stamp.nanosec,
                                            'index': tick.index, 'lateness_ns': tick.lateness_ns,
                                            'missed_before': tick.missed_before, 'clock_msg_ns': last_clock_msg[0],
                                            'wall_ns': time.monotonic_ns()}
                        requests.append(state['request'])
            elif last_command is None or now-last_command >= .019:
                command = AckermannDriveStamped()
                command.header.stamp = node.get_clock().now().to_msg()
                command.header.frame_id = 'base_link'
                speed, steering = profile.at(elapsed) if profile else (request_speed(elapsed), 0.)
                command.drive.speed, command.drive.steering_angle = speed, steering
                publisher.publish(command)
                state['request'] = {'sim_s': now, 'speed_mps': speed, 'steering_rad': steering}
                requests.append(state['request'])
                last_command = now
            if elapsed >= stop_elapsed and stop_future is None:
                if not stop_client.service_is_ready():
                    raise RuntimeError('STOP service unavailable')
                stop_future = stop_client.call_async(Trigger.Request())
                report['stop_request_sim_s'] = now
            if elapsed >= finish_elapsed:
                report['finish_wall_s'] = time.monotonic()-began
                report['finish_sim_s'] = now
                report['stop_acknowledged'] = stop_future.done() and stop_future.result().success
                report['final_mcu'] = mcu
                break
            if last_print is None or time.monotonic()-last_print > 20:
                print(f"{args.surface}: elapsed={elapsed:.3f} body={data['truth_longitudinal_mps']:.3f} wheel={data['wheel_surface_speed_mps']:.3f}", flush=True)
                last_print = time.monotonic()
        active = [r for r in trace if start <= r['sim_time_s'] < start+stop_elapsed]
        final = [r for r in trace if start+finish_elapsed-.5 <= r['sim_time_s'] <= start+finish_elapsed]
        report.update(peak_body_speed_mps=max(abs(r['truth_longitudinal_mps']) for r in active),
                      feedback_names=sorted(set(tuple(n) for n in feedback)), feedback_samples=len(feedback))
        report['input_valid'] = bool(feedback and active and final) and all(
            r['mcu']['feedback_mode'] == 'drive_motor_shaft' for r in active) and report['peak_body_speed_mps'] > .8
        report['mode_observed'] = bool(active) and all(
            r.get('evaluation_zero_speed_mode') == args.zero_speed_mode for r in active)
        report['input_valid'] &= report['mode_observed']
        if startup:
            report['startup_audit'] = audit_startup(startup.record, plugin_events)
            report['input_valid'] &= report['startup_audit']['passed']
        # 위 결합의 구성 요소를 따로 남긴다(판정식은 그대로). 기대 실패 분류에서 다른 실패를 숨기지 않기 위함.
        report['input_validity_components'] = {
            'feedback_present': bool(feedback), 'active_samples': bool(active), 'final_samples': bool(final),
            'motor_shaft_feedback': bool(active) and all(r['mcu']['feedback_mode'] == 'drive_motor_shaft' for r in active),
            'peak_body_speed_above_0p8': report['peak_body_speed_mps'] > .8, 'mode_observed': report['mode_observed']}
        if profile:
            if grid_spec:
                # 격자 모드 전용 감사. 기존 폴링 모드의 audit_requests 판정은 그대로 둔다.
                # 첫 판(scheduled_phase_v1 실행)은 아래 coverage·input_valid 갱신을 격자 모드에서 건너뛰었다.
                report['request_audit_mode'] = 'sim_grid_scheduled'
                report['request_audit'] = audit_scheduled_requests(
                    requests, sampler, start_ns, start_ns+sampler.duration_ns, grid_spec['period_ns'], grid_spec['phase_ns'])
                report['clock_observation'] = clock_summary(clock_values, report['command_schedule']['origin_ns'],
                                                            round(report['finish_sim_s']*1e9))
                # r2: 등록 창 밖 유지값 tick까지 같은 지연/누락 기준. 창 감사(request_audit)는 그대로 둔다.
                report['request_audit_all_ticks'] = audit_all_ticks(requests, grid_spec['period_ns'])
                report['input_valid'] &= report['request_audit_all_ticks']['passed']
                if trace_extra:
                    # 수신 스레드가 계속 붙일 수 있으므로 판정 시점 사본으로 감사한다(종료 후 전체는 따로 기록).
                    report['plugin_command_trace'] = command_trace_audit(list(plugin_events), dict(trace_audit))
                    report['input_valid'] &= report['plugin_command_trace']['valid']
            else:
                report['request_audit'] = audit_requests(requests, profile, start, start+profile.duration_s)
            gaps = [b['sim_time_s']-a['sim_time_s'] for a, b in zip(active, active[1:])]
            report['dynamics_coverage'] = {'samples': len(active), 'max_gap_s': max(gaps, default=None),
                'first_delay_s': active[0]['sim_time_s']-start if active else None,
                'last_delay_s': start+stop_elapsed-active[-1]['sim_time_s'] if active else None}
            report['dynamics_coverage']['passed'] = bool(gaps and min(gaps) > 0 and max(gaps) <= .05
                and report['dynamics_coverage']['first_delay_s'] <= .05
                and report['dynamics_coverage']['last_delay_s'] <= .05)
            report['final_stationary_coverage'] = bool(len(final) >= 20
                and final[-1]['sim_time_s']-final[0]['sim_time_s'] >= .45)
            report['input_valid'] &= (report['request_audit']['passed'] and report['dynamics_coverage']['passed']
                                      and report['final_stationary_coverage'] and report['drive_graph_valid'])
        report['stopped'] = bool(final) and all(math.hypot(r['truth_longitudinal_mps'], r['truth_lateral_mps']) < .05
                                               and abs(r['wheel_surface_speed_mps']) < .05 for r in final)
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        try:
            final_command = AckermannDriveStamped()
            final_command.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(final_command)
            if grid_spec:
                gc.callbacks.remove(on_gc)
            if injection is not None:
                # 주입 결과는 실제 발행 기록에서 읽기만 한다. 실패 판정은 위 감사가 그대로 계산한 값이다.
                report['evaluation_fault_injection'] = injection.outcome(requests, grid_spec['period_ns'])
            if process is not None:
                report['gazebo_stop_service'] = stop_gazebo_service(process, log_path.read_text(errors='replace'), partition)
            cleanup = stop_launch(process)
            if reader is not None:
                report['reader_forced_cleanup'] = stop_owned(reader)
            if trace_reader is not None:
                report['trace_reader_forced_cleanup'] = stop_owned(trace_reader)
                report['plugin_command_trace_final'] = command_trace_audit(list(plugin_events), dict(trace_audit))
            if stats_reader is not None:
                report['stats_reader_forced_cleanup'] = stop_owned(stats_reader)
        finally:
            gc_policy.restore()
            report['evaluation_gc'] = gc_policy.record
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        reader_log.close()
        report['shutdown_audit'] = audit_shutdown(log_path.read_text(errors='replace'), process.returncode if process else None,
                                                  cleanup.get('forced_cleanup'))
        report['remaining_pids'] = group_processes(process.pid) if process else {}
        report['shutdown_clean'] = bool(report['shutdown_audit']['clean'] and not report['remaining_pids']
                                       and report.get('gazebo_stop_service', {}).get('verified'))
        streams = [('braking_diagnostics', trace), ('wheel_contacts', contacts),
                   ('requested_commands', requests), ('safe_commands', safe_commands)]
        if args.evaluation_scheduled_apply:
            streams += [('scheduled_sent',scheduled_sent),('scheduled_events',scheduled_events),('scheduled_states',scheduled_states)]
            report['scheduled_apply_audit']=audit_scheduled(scheduled_sent,scheduled_events,scheduled_states)
            report['input_valid'] &= report['scheduled_apply_audit']['passed']
        if grid_spec:
            streams += [('actuation_status', mcu_statuses), ('output_status', output_statuses),
                        ('gc_events', gc_events), ('slow_callbacks', slow_callbacks)]
            if trace_extra:
                streams.append(('plugin_command_events', plugin_events))
            if schedule is not None:
                report['command_schedule']['diagnostics'] = schedule.diagnostics()
        for name, rows in streams:
            path = output/f'{name}.jsonl'
            path.write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
            report[name] = {'samples': len(rows), 'sha256': sha256(path)}
        if grid_spec:
            path = output/'evaluator_timing.i64'
            with path.open('wb') as stream:
                for name in TIMING_ARRAYS:
                    timing[name].tofile(stream)
            report['evaluator_timing'] = {
                'path': path.name, 'sha256': sha256(path), 'dtype': 'int64', 'byteorder': sys.byteorder,
                'arrays_in_order': {name: len(timing[name]) for name in TIMING_ARRAYS},
                'clock': 'wall=time.monotonic_ns (CLOCK_MONOTONIC), node clock=ROS sim time',
                'callback_stats': callback_stats,
                'scope': '평가기 자체 벽시계 계측. 원인 판정이 아니라 지연 위치를 좁히는 관측값'}
        report['contact_reader_audit'] = audit
        if startup:
            report['startup_reader_audit'] = startup_audit
            report['startup_audit_final'] = audit_startup(startup.record, plugin_events)
            report['input_valid'] &= (report['startup_audit_final']['passed']
                                      and startup_audit['handled_messages'] > 0
                                      and not startup_audit['parse_errors'] and not startup_audit['handler_errors'])
        report['source_unchanged'] = all(sha256(REPO/rel) == value for rel, value in sources.items())
        report['wall_elapsed_s'] = time.monotonic()-began
        report['passed'] = bool(report['input_valid'] and report['stopped'] and report['shutdown_clean']
                                and report.get('stop_acknowledged') and report['source_unchanged']
                                and report.get('final_mcu', {}).get('reason') == 'software_stop'
                                and audit['handled_messages'] > 0 and not audit['parse_errors'] and not audit['handler_errors'])
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps({k: report.get(k) for k in ('surface', 'input_valid', 'stopped', 'shutdown_clean', 'passed', 'error')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
