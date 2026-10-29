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
MODULES = {'arena_autonomy': ('boundary_evidence',
                    'core',
                    'corner_corridor',
                    'corner_distance',
                    'corner_evidence',
                    'corridor_observer',
                    'follow_trace',
                    'lateral_intent',
                    'lidar_motion',
                    'lidar_motion_ros',
                    'lidar_observation',
                    'lidar_safety',
                    'linear_observation',
                    'local_follow',
                    'local_path',
                    'local_pursuit',
                    'local_scene',
                    'm3_execution_trial',
                    'm3_observed_follow',
                    'observation_lateral',
                    'observation_support',
                    'observed_lateral_trial',
                    'occupancy_timing',
                    'parallel_lane_trial', 'partial_observation', 'partial_surface_alignment', 'pass_release_trial', 'traffic_recovery_trial',
                    'traffic_path_recovery_trial', 'corridor_roles', 'recovery_geometry', 'surface_anchor_trial', 'surface_reobservation', 'anchor_clearance_trial', 'bypass_wall_fit', 'bypass_entry_trial', 'bypass_entry_follow', 'bypass_handoff', 'same_path_gap_trial', 'rear_fringe_trial',
                    'polygon_batch',
                    'r0_plus',
                    'race_permission',
                    'race_session',
                    'race_start', 'race_start_local',
                    'rounded_occupancy_candidate',
                    'scan_topology',
                    'scene_motion',
                    'vision_process',
                    'wall_follow'),
 'arena_vehicle_interface': ('actuation_contract',
                             'actuation_ros',
                             'actuation_wire',
                             'applied_guard',
                             'drive_feedback',
                             'node_lifecycle',
                             'queued_log',
                             'timing_probe')}
SOURCES = [f'src/{package}/{package}/{name}.py'
           for package, names in MODULES.items() for name in names]
FIXTURE_SOURCES = ['scripts/audit_start_signal_layout.py', 'scripts/pipeline_mcu_rejection_probe.py',
                   'scripts/pipeline_restart_probe.py']
PROVENANCE = ['tests/test_pipeline_fault_validation.py', 'src/arena_description/config/vehicle.yaml',
              'config/tracks/official_v2026.10.24.yaml']
ZERO_TOPICS = ('applied', 'sink', 'mcu')
ZERO_GAP_SIM_S, ZERO_GAP_WALL_S = .25, 3.
STARTUP_GRAPH_WALL_S = 5.
# rmw graph cache가 endpoint를 먼저 알고 노드 이름을 아직 모를 때 쓰는 값.
UNKNOWN_NODE_NAME, UNKNOWN_NODE_NAMESPACE = '_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_'
# 제어기 재시작 구간 진단 전용. 판정에 쓰지 않으며 tick 4회마다 /proc 표본 한 번.
TRACE_SAMPLE_TICKS = 4
TRACE_FIELDS = dict(
    mcu=('state', 'reason', 'generation', 'accepted_seq', 'target_speed_mps', 'deadline_us', 'now_us',
         'feedback_seq', 'feedback_capture_us', 'feedback_valid', 'output_ready', 'stationary_ready'),
    sink=('now_us', 'speed_mps', 'blocked', 'reason', 'epoch', 'last_status_us'),
    safety=('reason', 'safe_speed_mps'),
    autonomy=('state', 'reason', 'started', 'signal', 'sim_time_s', 'speed_command_mps'))
PROBE_PREFIXES = ('HOST_DRIVE_DECISION=', 'MCU_FAULT_CONTEXT=', 'MCU_ENCODER_REJECTION=', 'SLOW_PUBLISH=')
PARSED_PREFIXES = ('MCU_FAULT_CONTEXT=', 'SLOW_PUBLISH=')
# 스레드별 표본은 비용 때문에 tick 8회마다. 느린 발행/spin 기록 한도는 진단 전용이다.
TRACE_THREAD_TICKS = 8
SLOW_SPIN_S = .015
ROLES = {
    'controller': ('arena_autonomy.local_pursuit', 'main', 'local_pursuit'),
    'safety': ('arena_autonomy.lidar_safety', 'main', 'lidar_safety'),
    'host': ('arena_vehicle_interface.actuation_ros', 'host_main', 'actuation_host'),
    'mcu': ('arena_vehicle_interface.actuation_ros', 'mcu_main', 'virtual_mcu'),
    'sink': ('arena_vehicle_interface.actuation_ros', 'actuator_main', 'guarded_sim_actuator'),
}


def child_bootstrap(role):
    """실제 import 해시를 기록하고 시험 자식에만 진단을 연결한다.

    모든 자식: 느린 발행 기록. MCU/호스트: 명령 경로 진단. 운영 launch에는 연결하지 않는다.
    """
    module, entry, _ = ROLES[role]
    rejection_probe = (
        f'import sys; sys.path.insert(0,{str(ROOT / "scripts")!r}); '
        'from pipeline_restart_probe import install_test_publish_timing_probe; '
        'from rclpy.publisher import Publisher; '
        'install_test_publish_timing_probe(Publisher); '
        'print("SLOW_PUBLISH_PROBE=validator_child_only",flush=True); '
    )
    if role == 'mcu':
        rejection_probe += (
            'from pipeline_mcu_rejection_probe import install_test_encoder_probe; '
            'from pipeline_restart_probe import install_test_mcu_command_probe; '
            'from arena_vehicle_interface.actuation_contract import ActuationReceiver; '
            'install_test_encoder_probe(ActuationReceiver); '
            'install_test_mcu_command_probe(ActuationReceiver); '
            'print("MCU_ENCODER_REJECTION_PROBE=validator_child_only",flush=True); '
            'print("MCU_FAULT_CONTEXT_PROBE=validator_child_only",flush=True); '
        )
    elif role == 'host':
        rejection_probe += (
            'from pipeline_restart_probe import install_test_host_drive_probe; '
            'from arena_vehicle_interface.actuation_ros import ActuationHost; '
            'install_test_host_drive_probe(ActuationHost); '
            'print("HOST_DRIVE_DECISION_PROBE=validator_child_only",flush=True); '
        )
    return (
        'import hashlib,importlib,json; '
        f'names={repr([f"{p}.{n}" for p, ns in MODULES.items() for n in ns])}; '
        'mods=[importlib.import_module(n) for n in names]; '
        'print("RUNTIME_SOURCE_MANIFEST="+json.dumps({m.__name__:{"path":m.__file__, '
        '"sha256":hashlib.sha256(open(m.__file__,"rb").read()).hexdigest()} for m in mods}),flush=True); '
        + rejection_probe + f'from {module} import {entry}; {entry}()'
    )


def controller_profile_parameters(profile):
    """시험에만 명시 적용한다. 기본 단독 주행/운영 launch는 바꾸지 않는다."""
    if profile == 'r0':
        return {}
    if profile == 'g1_per_ray_candidate':
        return dict(g1_follow_enabled=True, g1_boundary_evidence_mode='fragmented_candidate',
                    g1_occupancy_timing_mode='per_ray_candidate')
    if profile == 'g1_recent_observation_candidate':
        return dict(g1_follow_enabled=True, g1_boundary_evidence_mode='corner_distance_candidate',
                    g1_occupancy_timing_mode='per_ray_candidate', g1_corridor_fallback_enabled=True,
                    g1_recent_observation_enabled=True)
    raise ValueError('unknown controller test profile')


def g1_candidate_observed(status):
    """명령행 지정만으로 시험 모드가 실제 실행됐다고 간주하지 않는다."""
    return (status.get('mode') == 'g1_follow'
            and status.get('occupancy_timing_mode') == 'per_ray_candidate'
            and status.get('unfiltered_occupancy') is True
            and type(status.get('occupied_points')) is int and status['occupied_points'] > 0
            and status.get('occupancy_padding_points') == status['occupied_points'])


def apply_scan_fault(ranges, fault):
    """500점 fixture의 전방 0도 반환 하나만 무응답으로 만든다. 빈 공간 아님."""
    if fault not in ('none', 'scan_drop', 'imu_drop', 'front_missing', 'side_missing'):
        raise ValueError('unsupported sensor fixture fault')
    result = list(ranges)
    if fault in ('front_missing', 'side_missing'):
        if len(result) != 500:
            raise ValueError('front fault requires 500-ray fixture')
        result[250 if fault=='front_missing' else 375] = math.inf
    return result


def stamp_parts(seconds):
    return divmod(round(seconds * 1e9), 1_000_000_000)


def signal_rgb(color, mode='temporary_3light'):
    """테스트 모듈에 의존하지 않는 실제 픽셀 신호등 (rgb8)."""
    if mode == 'official_4red_candidate':
        from audit_start_signal_layout import light_image
        reds = {'red1': 1, 'red2': 2, 'red3': 3, 'red': 4, 'green': 0, 'off': 0}[color]
        return light_image(reds, color == 'green', diameter=12, paired=True).tobytes()
    if mode != 'temporary_3light':
        raise ValueError('unsupported signal fixture mode')
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


def proc_sample(pid):
    """주 스레드 실행/대기 누적·커널 대기 위치·페이지 폴트. 진단 전용이며 판정 입력이 아니다."""
    base = Path('/proc') / str(pid)
    raw = (base / 'stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    sample = dict(pid=pid, state=fields[0], minflt=int(fields[7]), majflt=int(fields[9]),
                  utime_ticks=int(fields[11]), stime_ticks=int(fields[12]), threads=int(fields[17]),
                  start_ticks=int(fields[19]))
    try:
        run_ns, wait_ns, slices = (int(value) for value in (base / 'schedstat').read_text().split())
        sample.update(main_run_ns=run_ns, main_runqueue_wait_ns=wait_ns, main_slices=slices)
    except (OSError, ValueError):
        pass
    try:
        sample['wchan'] = (base / 'wchan').read_text().strip() or None
    except OSError:
        pass
    return sample


def pressure_sample():
    """시스템 PSI 누적(µs). 커널이 제공하지 않으면 빈 값이며 원인 판정이 아니다."""
    result = {}
    for kind in ('cpu', 'io', 'memory'):
        try:
            for line in (Path('/proc/pressure') / kind).read_text().splitlines():
                scope, *items = line.split()
                result[kind + '_' + scope + '_total_us'] = int(dict(item.split('=') for item in items)['total'])
        except (OSError, KeyError, ValueError):
            pass
    return result


def compact_row(row):
    value = row['value']
    if isinstance(value, dict):
        value = {key: value[key] for key in TRACE_FIELDS.get(row['topic'], ()) if key in value}
    return dict(seq=row['sequence'], topic=row['topic'], wall_s=row['received_wall_s'],
                sim_s=row['received_sim_s'], source_sim_s=row['source_sim_s'], value=value)


def shm_listing():
    """DDS 공유 메모리 파일 이름·크기. 이전 실행 잔여물 대조용이며 삭제하지 않는다."""
    try:
        return sorted([path.name, path.stat().st_size] for path in Path('/dev/shm').iterdir())
    except OSError as error:
        return [['error', repr(error)]]


def probe_log_rows(lines, limit=32):
    """시험 전용 진단 줄 수와 MCU 고장 문맥·느린 발행 일부. 다른 줄은 해석하지 않는다."""
    counts = {prefix: 0 for prefix in PROBE_PREFIXES}
    parsed = {prefix: [] for prefix in PARSED_PREFIXES}
    for line in lines:
        for prefix in PROBE_PREFIXES:
            if line.startswith(prefix):
                counts[prefix] += 1
                if prefix in parsed and len(parsed[prefix]) < limit:
                    parsed[prefix].append(json.loads(line[len(prefix):]))
    return counts, parsed


def proc_threads(pid, names):
    """스레드별 상태·주 실행/대기 누적. 이름은 tid별로 한 번만 읽는다. 진단 전용."""
    rows = []
    for path in (Path('/proc') / str(pid) / 'task').iterdir():
        try:
            tid = int(path.name)
            key = (pid, tid)
            if key not in names:
                names[key] = (path / 'comm').read_text().strip()
            raw = (path / 'stat').read_text()
            run_ns, wait_ns, _ = (int(value) for value in (path / 'schedstat').read_text().split())
            rows.append(dict(tid=tid, comm=names[key], state=raw[raw.rfind(')') + 2], run_ns=run_ns,
                             runqueue_wait_ns=wait_ns))
        except (OSError, ValueError):
            continue
    return rows


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
                               maximum_sim_gap_s=0., maximum_wall_gap_s=0.,
                               per_topic={key: dict(maximum_sim_gap_s=0., maximum_wall_gap_s=0.,
                                                    sim_gap_end_sequence=None, sim_gap_end_received_sim_s=None)
                                          for key in ZERO_TOPICS})
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
                topic = watch['per_topic'][key]
                if sim_gap > topic['maximum_sim_gap_s']:
                    topic.update(maximum_sim_gap_s=sim_gap, sim_gap_end_sequence=row['sequence'],
                                 sim_gap_end_received_sim_s=row['received_sim_s'])
                topic['maximum_wall_gap_s'] = max(topic['maximum_wall_gap_s'], wall_gap)
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
            maximum_sim_gap_s=watch['maximum_sim_gap_s'], maximum_wall_gap_s=watch['maximum_wall_gap_s'],
            per_topic={key: dict(value) for key, value in watch['per_topic'].items()})

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
        log_path = self.output / f'{role}_{len(self.children)}.log'
        log = log_path.open('w', encoding='utf-8')
        # 각 자식의 실제 import 위치·해시도 수집한다.
        bootstrap = child_bootstrap(role)
        parameters = dict(use_sim_time=True)
        if role in ('controller', 'safety'):
            parameters.update(lidar_compensation='both', motion_scan_timing_verified=True,
                              motion_imu_topic='/imu/data', motion_feedback_mode='drive_motor_shaft',
                              motion_gear_ratio=15., motion_wheel_radius_m=.0325,
                              motion_lidar_x_m=.06, motion_rear_axle_x_m=-.0675,
                              wheelbase_m=.135, max_steering_angle_rad=.37)
            if self.report.get('controller_profile') == 'g1_recent_observation_candidate':
                parameters.update(controller_profile_parameters(self.report['controller_profile']))
        if role == 'controller':
            parameters.update(vision_execution='process', max_speed_mps=.6,
                              target_wall_distance_m=.425, lidar_x_m=.06,
                              image_timeout_s=1., control_rate_hz=50.)
            parameters.update(controller_profile_parameters(self.report.get('controller_profile', 'r0')))
            if self.report.get('signal_mode', 'temporary_3light') != 'temporary_3light':
                parameters['start_signal_mode'] = self.report['signal_mode']
            if self.report.get('scenario') == 'race_session':
                parameters['race_control_mode'] = 'b3_candidate'
        if role == 'mcu':
            parameters.update(feedback_mode='drive_motor_shaft', wheel_radius_m=.0325,
                              ticks_per_revolution=28, gear_ratio=15.)
        if role == 'sink':
            parameters['wheelbase_m'] = .135
        command = [sys.executable, '-c', bootstrap, '--ros-args']
        for name, value in parameters.items():
            command.extend(['-p', f'{name}:={str(value).lower() if isinstance(value, bool) else value}'])
        if role == 'controller' and self.report.get('scenario') == 'race_session':
            # 시험 전용: 제어기의 MCU 상태 입력만 시험기 중계로 받아 지연 전달을 주입한다.
            command.extend(['-r', '/actuation/status:=/validator/controller_mcu_status'])
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
    def __init__(self, output, wall_cap=480., sleep_s=.005, scenario='full', cycles=1,
                 signal_mode='temporary_3light', controller_profile='r0'):
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
        self.scenario, self.cycles, self.signal_mode = scenario, cycles, signal_mode
        controller_profile_parameters(controller_profile)
        self.controller_profile, self.sensor_fault = controller_profile, 'none'
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
                           scenario=scenario, cycles=cycles, signal_mode=signal_mode,
                           controller_profile=controller_profile, mcu_deadline_diagnostics=[])
        # 검사기 자신의 발행 지연도 같은 한도로 기록한다. 판정 입력이 아니다.
        from pipeline_restart_probe import install_test_publish_timing_probe
        from rclpy.publisher import Publisher
        slow = self.report['validator_slow_publishes'] = []
        self.restore_publish_probe = install_test_publish_timing_probe(
            Publisher, lambda row: slow.append(dict(row, scope='validator_process', sim_s=self.t)))
        self.children = OwnedProcesses(output, self.report)
        self.node = rclpy.create_node('pipeline_fault_validator',
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        topics = dict(clock='/clock', image='/camera/color/image_raw', scan='/scan',
                      imu='/imu/data', motor='/drive_motor/encoder')
        self.publishers = {key: self.node.create_publisher(self.types[key], topic,
                          qos_profile_sensor_data if key in ('scan', 'imu') else 100)
                           for key, topic in topics.items()}
        self.relay_pub, self.relay_hold = None, None
        if scenario == 'race_session':
            self.relay_pub = self.node.create_publisher(String, '/validator/controller_mcu_status', 1000)
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
                         ('stop', '/actuation/stop'), ('reset', '/autonomy/reset'),
                         ('disarm', '/actuation/disarm')]}
        from std_srvs.srv import SetBool
        self.SetBool = SetBool
        self.enable_client = self.node.create_client(SetBool, '/autonomy/enable')
        self.race_results, self.race_request = [], 0
        if scenario == 'race_session':
            # 경기 감독 명령은 사람 운영 도구를 대신하는 시험 입력이다. 정답/신호 상태를 담지 않는다.
            self.race_pub = self.node.create_publisher(String, '/race/command', 10)
            self.node.create_subscription(String, '/race/command_result',
                                          lambda m: self.race_results.append(json.loads(m.data)), 10)
        self.color, self.images, self.held_stamp = 'green', True, None
        colors = ('red', 'green', 'red1', 'red2', 'red3', 'off') if signal_mode == 'official_4red_candidate' else ('red', 'green')
        self.rgb = {name: signal_rgb(name, signal_mode) for name in colors}
        self.last = {key: -math.inf for key in ('scan', 'image', 'motor')}
        self.last_counts = None
        self.last_motor_t = None
        self.trace, self.traces = None, []

    def on_status(self, key, message):
        if key == 'mcu' and getattr(self, 'relay_pub', None) is not None:
            if self.relay_hold is not None:
                self.relay_hold.append(message)
            else:
                self.relay_pub.publish(message)
        value = json.loads(message.data)
        stamp = value.get('sim_time_s') if key == 'autonomy' else (
            value.get('now_us', 0) * 1e-6 if key in ('mcu', 'sink') else None)
        self.monitor.receive(key, value, self.t, stamp)

    def data(self, key):
        return self.monitor.latest.get(key, {}).get('value', {})

    def release_relay(self):
        held, self.relay_hold = self.relay_hold or [], None
        for message in held:
            self.relay_pub.publish(message)
        self.monitor.events.append(dict(kind='relay_release', count=len(held), wall_s=time.monotonic(), sim_s=self.t))

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
        self.trace_tick()
        self.plant.step(float(self.data('applied') or 0.))
        self.history.append((self.t, self.plant.distance))
        clock = self.types['clock']()
        clock.clock.sec, clock.clock.nanosec = stamp_parts(self.t)
        self.publishers['clock'].publish(clock)
        imu = self.stamp(self.types['imu'](), self.t)
        if self.sensor_fault != 'imu_drop':
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
        if self.sensor_fault != 'scan_drop' and self.t - self.last['scan'] >= .1 - 1e-8:
            scan = self.stamp(self.types['scan'](), self.t - .1)
            scan.header.frame_id = 'laser_frame'
            scan.angle_min, scan.angle_increment = -math.pi, math.tau / 500
            scan.angle_max = scan.angle_min + 499 * scan.angle_increment
            scan.range_min, scan.range_max = .05, 12.
            scan.time_increment, scan.scan_time = .1 / 500, .1
            scan.ranges = apply_scan_fault([corridor_range(scan.angle_min + i * scan.angle_increment,
                .06 + self.position_at(self.t - .1 + i * .1 / 500) - self.plant.distance)
                           for i in range(500)], self.sensor_fault)
            self.publishers['scan'].publish(scan)
            self.last['scan'] = self.t
        if self.images and self.t - self.last['image'] >= .1 - 1e-8:
            image = self.stamp(self.types['image'](), self.t if self.held_stamp is None else self.held_stamp)
            image.header.frame_id = 'camera_color_optical_frame'
            image.height, image.width, image.step, image.encoding = 480, 848, 848 * 3, 'rgb8'
            image.data = self.rgb[self.color]
            self.publishers['image'].publish(image)
            self.last['image'] = self.t
        spin_began = time.monotonic()
        for _ in range(24):
            self.rclpy.spin_once(self.node, timeout_sec=0)
        trace = getattr(self, 'trace', None)
        if trace is not None and time.monotonic() - spin_began >= SLOW_SPIN_S:
            trace.setdefault('slow_spins', []).append(dict(start_wall_s=spin_began, sim_s=self.t,
                                                          duration_s=time.monotonic() - spin_began))
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
        expected = {key: lambda v: v > .4 for key in ('drive', 'safe', 'applied')}
        if self.controller_profile in ('g1_per_ray_candidate', 'g1_recent_observation_candidate'):
            expected['autonomy'] = g1_candidate_observed
        self.wait(lambda: self.plant.speed > .4 and all(self.fresh(key, case['sequence'],
                  predicate) for key, predicate in expected.items()), name)
        self.finish(case, expected=expected, duration=.1)
        return case

    def red_green(self, name='fresh_red_to_green_recovery', expect_motion=True):
        self.images, self.held_stamp, self.color = True, None, 'red'
        baseline = self.monitor.sequence
        observed = 'red4_confirmed' if self.signal_mode == 'official_4red_candidate' else 'red'
        self.wait(lambda: self.fresh('autonomy', baseline, lambda v: v['signal'] == observed), 'actual red pixels')
        self.pump(.8)
        self.color = 'green'
        if expect_motion:
            self.moving(name)

    def restart_controller(self):
        """지역 제어기만 교체한다. 보호층의 신선한 0 명령으로 MCU 허가는 유지돼야 한다.

        MCU ARMED 유지 확인은 기존 다음 단계(재허가 없는 빨강→초록 출발)의 전제를
        이 사례에서 명시한 것이다. 고장 고정이 생기면 CLEAR/ARM으로 덮지 않고 실패로 남긴다.
        """
        def restart():
            self.children.stop('controller')
            self.color, self.images, self.held_stamp = 'green', True, None
            self.children.start('controller')
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline,
                lambda v: not v['started'] and v['vision']['stats']['accepted'] >= 3),
                'restart accepts green pixels without start permission')
            self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
        self.begin_trace('controller_restart_%d' % (len(getattr(self, 'traces', [])) + 1))
        self.stop_case('controller_restart_green_only_no_start', restart,
                       {'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.,
                        'mcu': lambda v: v['state'] == 'ARMED'}, no_resume=True)
        self.red_green()
        self.end_trace('passed')

    def begin_trace(self, name):
        if getattr(self, 'trace', None) is not None:
            raise RuntimeError('restart trace already active')
        self.trace = dict(name=name, start_sequence=self.monitor.sequence, start_wall_s=time.monotonic(),
                          start_sim_s=self.t, ticks=[], process_samples=[],
                          interpretation='diagnostic receipts/ticks/proc samples only; not a pass criterion')

    def trace_tick(self):
        trace = getattr(self, 'trace', None)
        if trace is None:
            return
        trace['ticks'].append((time.monotonic(), self.t))
        if len(trace['ticks']) % TRACE_SAMPLE_TICKS == 1:
            trace['process_samples'].append(self.sample_processes(
                threads=len(trace['ticks']) % TRACE_THREAD_TICKS == 1))

    def sample_processes(self, threads=False):
        sample = dict(wall_s=time.monotonic(), sim_s=self.t, pressure=pressure_sample(), processes=[])
        names = self.__dict__.setdefault('thread_names', {})
        targets = [('validator', os.getpid(), None)]
        targets += [(o['record']['role'], o['record']['pid'], o['record']['identity'])
                    for o in self.children.children if o['child'].poll() is None]
        targets += [('vision', v['pid'], v) for v in self.children.vision]
        for role, pid, identity in targets:
            try:
                row = proc_sample(pid)
                if identity is not None and row['start_ticks'] != identity['start_ticks']:
                    continue
                if threads:
                    row['thread_rows'] = proc_threads(pid, names)
                sample['processes'].append(dict(role=role, **row))
            except (OSError, ValueError, IndexError):
                continue
        return sample

    def end_trace(self, outcome):
        trace, self.trace = getattr(self, 'trace', None), None
        if trace is None:
            return None
        rows = [row for row in self.monitor.rows if row['sequence'] > trace['start_sequence']]
        trace.update(outcome=outcome, end_wall_s=time.monotonic(), end_sim_s=self.t,
                     end_sequence=self.monitor.sequence,
                     rows_complete=not rows or rows[0]['sequence'] == trace['start_sequence'] + 1,
                     rows=[compact_row(row) for row in rows])
        if not hasattr(self, 'traces'):
            self.traces = []
        self.traces.append(trace)
        return trace

    def graph_snapshot(self):
        expected_publishers = {'/drive': 'local_pursuit', '/drive/safe': 'lidar_safety',
                               '/sim/cmd_vel': 'guarded_sim_actuator'}
        race_mode = getattr(self, 'scenario', None) == 'race_session'
        if race_mode:
            expected_publishers['/race/command_result'] = 'local_pursuit'
            expected_publishers['/validator/controller_mcu_status'] = 'pipeline_fault_validator'
        audit = dict(publishers={}, subscriptions={})
        # 시작 감사 전용 분류. discovery는 아직 발견되지 않은 것, definite는 이미 관측된 잘못된 그래프다.
        discovery, definite = [], []
        for topic, owner in expected_publishers.items():
            audit['publishers'][topic] = [dict(node=info.node_name, namespace=info.node_namespace,
                gid=list(info.endpoint_gid)) for info in self.node.get_publishers_info_by_topic(topic)]
        for topic in ('/clock', '/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data') + (
                ('/race/command', '/race/command_result', '/validator/controller_mcu_status') if race_mode else ()):
            audit['publishers'][topic] = [dict(node=info.node_name, namespace=info.node_namespace,
                gid=list(info.endpoint_gid)) for info in self.node.get_publishers_info_by_topic(topic)]
        sensor_topics = {'/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data',
                         '/clock', '/parameter_events'}
        race_inputs = {'/validator/controller_mcu_status', '/race/command'} if race_mode else set()
        for name, allowed in [('local_pursuit', sensor_topics | race_inputs),
                              ('lidar_safety', sensor_topics - {'/camera/color/image_raw'} | {'/drive'})]:
            try:
                topics = {t for t, _ in self.node.get_subscriber_names_and_types_by_node(name, '/')}
            except Exception as error:
                # endpoint 발견과 노드 이름 조회 갱신은 동시에 끝나지 않을 수 있다. 미수렴으로만 기록한다.
                if type(error).__name__ != 'NodeNameNonExistentError':
                    raise
                audit.setdefault('missing_nodes', []).append(name)
                audit['subscriptions'][name] = None
                audit.setdefault('input_checks', {})[name] = False
                discovery.append('node_name_missing:' + name)
                continue
            audit['subscriptions'][name] = sorted(topics)
            required = {'/scan', '/clock', '/drive_motor/encoder', '/imu/data'} | (
                {'/camera/color/image_raw'} | race_inputs if name == 'local_pursuit' else {'/drive'})
            audit.setdefault('input_checks', {})[name] = not topics - allowed and required <= topics
            definite.extend(f'unexpected_subscription:{name}:{topic}' for topic in sorted(topics - allowed))
            discovery.extend(f'subscription_missing:{name}:{topic}' for topic in sorted(required - topics))
        expected_publishers.update({topic: 'pipeline_fault_validator' for topic in
            ('/clock', '/scan', '/camera/color/image_raw', '/drive_motor/encoder', '/imu/data')
            + (('/race/command',) if race_mode else ())})
        audit['valid'] = all([entry['node'] for entry in audit['publishers'][topic]] == [owner]
            and all(entry['namespace'] == '/' for entry in audit['publishers'][topic])
            for topic, owner in expected_publishers.items()) and all(audit['input_checks'].values())
        for topic, owner in expected_publishers.items():
            entries = audit['publishers'][topic]
            if len(entries) > 1:
                definite.append('duplicate_publisher:' + topic)
            elif not entries:
                discovery.append('publisher_missing:' + topic)
            elif UNKNOWN_NODE_NAME in (entries[0]['node'], entries[0]['namespace']) or \
                    UNKNOWN_NODE_NAMESPACE in (entries[0]['node'], entries[0]['namespace']):
                discovery.append('publisher_name_unresolved:' + topic)
            elif entries[0]['node'] != owner or entries[0]['namespace'] != '/':
                definite.append('wrong_publisher_owner:' + topic)
        audit['startup_problems'] = dict(discovery=discovery, definite=definite)
        return audit

    def graph_audit(self, convergence_wall_s=0., startup=False):
        """startup=True는 발견 준비 부족만 제한 대기한다. 기본(최종 감사) 수렴 동작은 그대로다."""
        observation = dict(started_wall_s=time.monotonic(), convergence_wall_cap_s=convergence_wall_s,
                           policy='startup_discovery_only' if startup else 'converge_any_invalid',
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
            if startup and audit['startup_problems']['definite']:
                # 이미 관측된 중복·잘못된 소유자·예상외 구독은 잠시 뒤 사라져도 통과시키지 않는다.
                observation['startup_definite_failure'] = audit['startup_problems']['definite']
                raise AssertionError('startup graph has invalid observed endpoints: ' + signature)
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
        # 시작 그래프도 정확한 단일 발행자·입력이어야 한다. 이름 미갱신·필요 endpoint 부족만 제한 대기한다.
        self.graph_audit(convergence_wall_s=STARTUP_GRAPH_WALL_S, startup=True)
        self.wait(lambda: self.stopped(0), 'fresh startup zero outputs')
        if self.scenario == 'race_session':
            # 시작 뒤 ARM을 목격하려면 제어기가 ARM 전의 fence 이후 상태를 먼저 받아야 한다.
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline, lambda v: bool((v.get('race') or {}).get(
                'rearm_baseline'))), 'controller observed post-start MCU state before ARM')
        self.stop_case('boot_disarmed', lambda: None, {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
        self.stop_case('armed_green_only_cannot_start', lambda: self.call('arm'),
                       {'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.}, no_resume=True)
        if self.scenario == 'start_contract':
            self.run_start_contract()
            return
        if self.scenario == 'race_session':
            self.run_race_session()
            return
        self.red_green()
        if self.controller_profile == 'g1_recent_observation_candidate' and self.scenario == 'full':
            self.run_recent_observation()
        if self.controller_profile in ('g1_per_ray_candidate', 'g1_recent_observation_candidate') and self.scenario == 'full':
            self.run_g1_sensor_faults()
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

    def run_recent_observation(self):
        """별도 센서/운동 입력을 쓰는 두 노드가 실제 측방 보완을 실행하는지 확인한다."""
        if self.report.get('recent_check_mode') == 'evidence_expiry':
            return self.run_recent_observation_evidence_expiry()
        case = self.mark('recent_side_observation_independent_cap')
        self.sensor_fault = 'side_missing'
        def limited(v): return 0.05 < v <= .200001
        expected = {key: limited for key in ('drive','safe','applied')}
        expected['autonomy'] = lambda v: (g1_candidate_observed(v) and
                                         v.get('recent_observation_selected') is True)
        expected['safety'] = lambda v: (v.get('recent_observation') or {}).get('supported') is True
        self.wait(lambda: all(self.fresh(key,case['sequence'],predicate) for key,predicate in expected.items()),
                  'independent recent observation nodes')
        self.finish(case, expected=expected, duration=.3)
        self.sensor_fault = 'none'
        self.moving('recent_side_observation_fresh_full_recovery')

    def run_recent_observation_evidence_expiry(self):
        """일시 지지/원시 관측 만료를 분리한다. 결측 중 연속 진행 합격은 아니다.

        기존 continuity 시나리오의 >.05 m/s 지속 기준과 실패는 유지한다.
        이 대조는 독립 두 노드가 같은 결측을 실제 검사했는지, 이후 새로운
        광선이 끊겼을 때 과거 지지만으로 계속 구동하지 않는지를 검사한다.
        """
        case=self.mark('recent_evidence_independent_positive_cap')
        self.sensor_fault='side_missing'
        def capped(value): return 0. < value <= .200001
        expected={key:capped for key in ('drive','safe','applied')}
        expected['autonomy']=lambda v: g1_candidate_observed(v) and v.get('recent_observation_selected') is True
        expected['safety']=lambda v: ((v.get('recent_observation') or {}).get('supported') is True and
            (v.get('front_observation') or {}).get('reason')=='front_unobserved')
        self.wait(lambda: all(self.fresh(k,case['sequence'],p) for k,p in expected.items()),
                  'independent positive recent evidence')
        case.update(passed=True,final_sim_s=self.t,final_wall_s=time.monotonic(),
                    evidence={k:self.monitor.latest[k] for k in expected},
                    continuous_progress_verified=False,minimum_progress_required=False)
        self.capture_zeros(case)
        print(case['name']+': passed (point evidence only)',flush=True)
        self.stop_case('recent_evidence_expiry_stops',lambda:setattr(self,'sensor_fault','scan_drop'),
            {'drive':lambda v:v==0.,'safe':lambda v:v==0.,
             'safety':lambda v:v.get('reason')=='stale_input'})
        self.sensor_fault='none'
        self.moving('recent_evidence_fresh_full_recovery')

    def run_g1_sensor_faults(self):
        """G1 후보에 기존 보호를 적용한다. 실차 제동이 아닌 출력·임시 모형 회귀."""
        reasons = {'scan_drop': ('stale_input',), 'front_missing': ('front_unobserved',),
                   'imu_drop': ('motion_stale', 'motion_gap')}
        for fault, accepted_reasons in reasons.items():
            stopped = self.stop_case('g1_'+fault+'_stop', lambda fault=fault: setattr(self, 'sensor_fault', fault),
                {'drive': lambda v: v == 0., 'safe': lambda v: v == 0.,
                 'safety': lambda v, rs=accepted_reasons: v['reason'] in rs})
            stopped['fixture_fault'] = fault
            self.sensor_fault = 'none'
            self.moving('g1_'+fault+'_fresh_recovery')

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

    def run_start_contract(self):
        """선택형 공식 픽셀→기존 ROS 보호/MCU 경계. 새 RacePermission은 미연결."""
        if self.signal_mode != 'official_4red_candidate':
            raise ValueError('start_contract requires official candidate')
        for color in ('off', 'red1', 'red2', 'red3', 'red'):
            self.stop_case('official_'+color+'_no_start', lambda color=color: setattr(self, 'color', color),
                           {'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.}, no_resume=True)
        self.stop_case('official_reds_off_without_green_no_start', lambda: setattr(self, 'color', 'off'),
                       {'autonomy': lambda v: not v['started']}, no_resume=True)
        self.red_green()
        self.color = 'off'
        self.moving('official_green_extinguished_keeps_race_start')
        self.stop_case('official_rgb_gap_stops', lambda: setattr(self, 'images', False),
                       {'autonomy': lambda v: v['state'] == 'SENSOR_STOP',
                        'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        self.images = True
        self.moving('official_fresh_off_image_restores_existing_permit')
        self.stop_case('official_controller_exit_stops_before_restart',
                       lambda: self.children.send('controller', signal.SIGINT, phase='cleanup'),
                       {'safe': lambda v: v == 0.})
        self.restart_controller()
        self.stop_case('official_explicit_stop_latches', lambda: self.call('stop'),
                       {'mcu': lambda v: v['state'] == 'ESTOP_LATCHED', 'sink': lambda v: v['blocked']})
        self.stop_case('official_clear_alone_is_disarmed', lambda: self.call('clear_stop'),
                       {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
        self.call('arm')
        self.moving('official_separate_rearm_after_existing_visual_start')
        self.final_stop()

    def call_result(self, name):
        """서비스 거절을 시험 실패로 바꾸지 않고 결과만 기록한다."""
        client = self.clients[name]
        self.wait(client.service_is_ready, name + ' service ready')
        future = client.call_async(self.Trigger.Request())
        self.wait(future.done, name + ' ACK')
        result = future.result()
        self.monitor.events.append(dict(kind='service', name=name, wall_s=time.monotonic(),
                                        sim_s=self.t, success=result.success, message=result.message))
        return result

    def enable(self, value):
        self.wait(self.enable_client.service_is_ready, 'autonomy enable ready')
        future = self.enable_client.call_async(self.SetBool.Request(data=value))
        self.wait(future.done, 'autonomy enable ACK')
        result = future.result()
        self.monitor.events.append(dict(kind='service', name='autonomy_enable_' + str(value).lower(),
            wall_s=time.monotonic(), sim_s=self.t, success=result.success, message=result.message))
        return result

    def race_state(self):
        return self.data('autonomy').get('race') or {}

    def race_send(self, payload, request=None):
        """`/race/command` 결과를 같은 요청 번호로 회수한다. 형식 오류 사례는 결과 개수로 기다린다."""
        self.wait(lambda: self.race_pub.get_subscription_count() >= 1, 'race command subscriber')
        before = len(self.race_results)
        text = payload if isinstance(payload, str) else json.dumps(payload)
        self.race_pub.publish(self.types_string(data=text))
        self.wait(lambda: any(r.get('request') == request for r in self.race_results[before:])
                  if request else len(self.race_results) > before, 'race command result')
        result = next(r for r in self.race_results[before:] if not request or r.get('request') == request)
        self.monitor.events.append(dict(kind='race_command', payload=text, result=result,
                                        wall_s=time.monotonic(), sim_s=self.t))
        return result

    def race_command(self, op, **fields):
        self.race_request += 1
        request = 'validator-%d' % self.race_request
        return self.race_send(dict(v=1, request=request, op=op, **fields), request)

    def race_ack(self, **override):
        state = self.race_state()
        fields = dict(session=state['session'], race_epoch=state['race_epoch'],
                      intervention=state['intervention'], hazards_cleared=True, operator_reviewed=True)
        fields.update(override)
        return self.race_command('acknowledge', **fields)

    def expect(self, result, accepted, reason=None):
        if not isinstance(result, dict):
            result = dict(success=result.success, message=result.message)
        ok = result.get('accepted', result.get('success')) is accepted and (
            reason is None or str(result.get('reason', result.get('message', ''))).startswith(reason))
        self.report.setdefault('race_expectations', []).append(dict(result=result, accepted=accepted,
                                                                     reason=reason, passed=ok, sim_s=self.t))
        if not ok:
            raise AssertionError('unexpected race result: ' + json.dumps(result, ensure_ascii=False))
        return result

    def run_race_session(self):
        """선택형 B3 경기 감독 v2: 시작/새 경기 handshake·같은 경기 재개·STOP 번호·지연 MCU 상태.

        STOP 판단(상위 래치), MCU STOP 배송/ACK, 가상 모형 정지를 각각 따로 기록한다.
        제어기의 MCU 상태 입력만 시험기 중계를 거치며 host/출력 보호는 원래 토픽을 쓴다.
        """
        if self.signal_mode != 'official_4red_candidate':
            raise ValueError('race_session requires official candidate')
        from std_msgs.msg import String
        self.types_string = String
        latched = lambda reason: (lambda v: v['state'] == 'RACE_LATCHED' and
                                  str(v['race']['latch']).startswith(reason))
        rearm_wait = lambda v: (v['state'] == 'ACTUATOR_WAIT' and v['race']['latch'] is None and
                                v['race']['actuator_reason'] == 'mcu_rearm_required')
        def lights_until(predicate, label):
            self.red_green(expect_motion=False)
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline, predicate), label)
        # 0) 프로세스 시작: 래치, 신호만으로 출발 불가. 시작 뒤 ARM(공통 접두 사례)과 사람 확인이 필요.
        first = self.race_state()
        if first['latch'] != 'process_start' or first['intervention'] != 1 or first['race_epoch'] != 0:
            raise AssertionError('process start is not latched: ' + json.dumps(first))
        self.stop_case('b3_startup_lights_without_ack_no_start',
                       lambda: lights_until(lambda v: latched('process_start')(v) and v['race']['visual_start_fact'],
                                            'startup lights stay latched'),
                       {'autonomy': latched('process_start'), 'mcu': lambda v: v['state'] == 'ARMED'}, no_resume=True)
        self.expect(self.race_ack(), True, 'policy_latch_cleared')
        self.moving('b3_startup_ack_with_post_start_arm_and_lights')
        self.color = 'off'
        self.moving('b3_green_extinguished_keeps_race_start')
        self.stop_case('b3_rgb_gap_sensor_hold', lambda: setattr(self, 'images', False),
                       {'autonomy': lambda v: v['state'] == 'SENSOR_STOP' and v['race']['latch'] is None,
                        'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        self.images = True
        before = self.race_state()['intervention']
        self.moving('b3_sensor_hold_automatic_resume')
        if self.race_state()['intervention'] != before:
            raise AssertionError('temporary sensor hold created an intervention')
        self.expect(self.enable(True), False, 'B3: enable cannot')
        self.moving('b3_legacy_enable_true_rejected_while_running')
        # 1) 상위 STOP만: MCU는 ARMED. 활성 STOP 재전송만 같은 개입, 해제 뒤에도 실제 재ARM 필요.
        stop_ids = {}
        def upper_stop():
            stop_ids['first'] = self.expect(self.race_command('stop'), True, 'stop_latched')['request']
        self.stop_case('b3_upper_stop_policy_latch_mcu_still_armed', upper_stop,
                       {'autonomy': latched('explicit_stop'), 'mcu': lambda v: v['state'] == 'ARMED'})
        def rejected_inputs():
            active = self.race_state()['intervention']
            self.expect(self.race_send(dict(v=1, request=stop_ids['first'], op='stop'), stop_ids['first']),
                        True, 'duplicate_stop_already_applied')
            if self.race_state()['intervention'] != active:
                raise AssertionError('active STOP resend created a new intervention')
            self.expect(self.enable(True), False, 'B3: enable cannot')
            self.expect(self.race_send('{not json'), False, 'invalid_payload')
            self.expect(self.race_ack(intervention=active - 1), False, 'acknowledge_rejected')
            self.expect(self.race_ack(session='0' * 32), False, 'session_mismatch')
        self.stop_case('b3_stop_resend_enable_stale_ack_and_malformed_no_resume', rejected_inputs,
                       {'autonomy': latched('explicit_stop')}, no_resume=True)
        ack_ids = []
        def ack_rearm_wait():
            ack_ids.append(self.expect(self.race_ack(), True, 'policy_latch_cleared')['request'])
        self.stop_case('b3_ack_without_mcu_rearm_no_resume', ack_rearm_wait,
                       {'autonomy': rearm_wait, 'mcu': lambda v: v['state'] == 'ARMED'}, no_resume=True)
        # 2) 래치 전 DISARM→ARM 상태를 제어기에 늦게 전달: 재ARM 증거가 아니다.
        def delayed_pre_latch_states():
            self.relay_hold = []
            self.call('disarm')
            self.call('arm')
            held = list(self.relay_hold)
            self.expect(self.race_command('stop'), True, 'stop_latched')
            state = self.race_state()
            self.report['delayed_status_injection'] = dict(held_messages=len(held),
                held_states=sorted({json.loads(m.data)['state'] for m in held}),
                held_generations=sorted({json.loads(m.data)['generation'] for m in held}),
                fence_s=state['rearm_fence_s'], before_release_counts=state['mcu_counts'],
                before_release_ignored=state['counts']['rearm_ignored_before_fence'])
            self.release_relay()
            self.pump(.3)
            self.expect(self.race_ack(), True, 'policy_latch_cleared')
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline, rearm_wait), 'delayed states are not re-ARM')
            after = self.race_state()
            self.report['delayed_status_injection'].update(after_release_counts=after['mcu_counts'],
                after_release_ignored=after['counts']['rearm_ignored_before_fence'],
                rearm_baseline=after['rearm_baseline'], mcu_generation=after['mcu']['generation'])
            if not {'DISARMED', 'ARMED'} <= set(self.report['delayed_status_injection']['held_states']):
                raise AssertionError('delayed injection did not hold both DISARM and ARM states')
        self.stop_case('b3_delayed_pre_latch_disarm_arm_not_rearm', delayed_pre_latch_states,
                       {'autonomy': rearm_wait}, no_resume=True)
        self.stop_case('b3_post_fence_disarm_no_resume', lambda: self.call('disarm'),
                       {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
        self.call('arm')
        self.moving('b3_same_race_resume_after_post_fence_rearm_without_new_lights')
        # 3) MCU STOP 직접: 상위 개입으로 변환, CLEAR/ARM만으로는 재개 불가.
        before = self.race_state()['intervention']
        self.stop_case('b3_mcu_stop_becomes_intervention', lambda: self.call('stop'),
                       {'mcu': lambda v: v['state'] == 'ESTOP_LATCHED', 'sink': lambda v: v['blocked'],
                        'autonomy': latched('mcu:ESTOP_LATCHED')})
        if self.race_state()['intervention'] != before + 1:
            raise AssertionError('MCU STOP was not one new intervention')
        def clear_arm():
            self.expect(self.call_result('arm'), False)
            self.call('clear_stop')
            self.call('arm')
        self.stop_case('b3_mcu_clear_arm_without_ack_no_resume', clear_arm,
                       {'mcu': lambda v: v['state'] == 'ARMED', 'autonomy': latched('mcu:ESTOP_LATCHED')},
                       no_resume=True)
        self.expect(self.race_ack(), True, 'policy_latch_cleared')
        self.moving('b3_same_race_resume_after_mcu_stop_ack')
        # 4) STOP 요청 번호: 다른 op 번호와 겹쳐도, 이미 해제된 STOP 번호여도 실제로 고정한다.
        def resume_after_ack_rearm(name):
            self.expect(self.race_ack(), True, 'policy_latch_cleared')
            self.pump(.2)
            self.call('disarm')
            self.call('arm')
            self.moving(name)
        self.stop_case('b3_stop_with_previous_ack_id_latches',
                       lambda: self.expect(self.race_send(dict(v=1, request=ack_ids[0], op='stop'), ack_ids[0]),
                                           True, 'stop_latched'),
                       {'autonomy': latched('explicit_stop')})
        resume_after_ack_rearm('b3_resume_after_colliding_stop')
        self.stop_case('b3_released_stop_id_resend_latches',
                       lambda: self.expect(self.race_send(dict(v=1, request=stop_ids['first'], op='stop'),
                                                          stop_ids['first']), True, 'stop_latched'),
                       {'autonomy': latched('explicit_stop')})
        resume_after_ack_rearm('b3_resume_after_released_stop_resend')
        # 5) 새 경기: 늦은/다른 세션 명령 거절. 정상 명령은 새 영상 세대·새 개입·사람 확인·재ARM·새 신호.
        state = self.race_state()
        self.expect(self.race_command('new_race', session=state['session'], race_epoch=state['race_epoch'] + 7),
                    False, 'race_epoch_mismatch')
        self.expect(self.race_command('new_race', session='0' * 32, race_epoch=state['race_epoch']),
                    False, 'session_mismatch')
        self.moving('b3_rejected_new_race_commands_keep_running')
        self.stop_case('b3_new_race_latches_and_requires_new_sequence',
                       lambda: self.expect(self.race_command('new_race', session=state['session'],
                                                             race_epoch=state['race_epoch']), True, 'new_race'),
                       {'autonomy': lambda v: latched('new_race')(v) and not v['race']['started'] and
                        v['race']['race_epoch'] == state['race_epoch'] + 1 and
                        v['race']['intervention'] == state['intervention'] + 1,
                        'mcu': lambda v: v['state'] == 'ARMED'})
        self.stop_case('b3_new_race_green_only_no_start', lambda: setattr(self, 'color', 'green'),
                       {'autonomy': lambda v: not v['race']['started'] and not v['race']['visual_start_fact']},
                       no_resume=True)
        def old_images():
            self.held_stamp = self.last['image']
            self.color = 'green'
        self.stop_case('b3_new_race_old_stamp_images_no_start', old_images,
                       {'autonomy': lambda v: not v['race']['started']}, no_resume=True)
        self.held_stamp = None
        def new_race_release():
            self.expect(self.race_ack(), True, 'policy_latch_cleared')
            lights_until(lambda v: v['race']['started'] and rearm_wait(v), 'new race waits for MCU re-ARM')
        self.stop_case('b3_new_race_ack_and_lights_still_need_rearm', new_race_release,
                       {'autonomy': rearm_wait}, no_resume=True)
        self.call('disarm')
        self.call('arm')
        self.moving('b3_new_race_start_after_ack_rearm_and_lights')
        # 6) 구형 reset: 새 경기 + 래치. 이전 경기 번호의 해제 거절.
        state = self.race_state()
        self.stop_case('b3_legacy_reset_fail_closed', lambda: self.call('reset'),
                       {'autonomy': latched('legacy_reset'), 'mcu': lambda v: v['state'] == 'ARMED'})
        if self.race_state()['race_epoch'] != state['race_epoch'] + 1:
            raise AssertionError('legacy reset did not start a new race epoch')
        def reset_release():
            self.expect(self.race_ack(race_epoch=state['race_epoch']), False, 'race_epoch_mismatch')
            self.expect(self.race_ack(), True, 'policy_latch_cleared')
            lights_until(lambda v: v['race']['started'] and rearm_wait(v), 'reset waits for MCU re-ARM')
        self.stop_case('b3_legacy_reset_ack_and_lights_still_need_rearm', reset_release,
                       {'autonomy': rearm_wait}, no_resume=True)
        self.call('disarm')
        self.call('arm')
        self.moving('b3_legacy_reset_resume_after_ack_lights_and_rearm')
        # 7) 영상 고정 고장 → 제어기 재시작: 새 session·시작 래치, 기존 ARMED 재사용 금지.
        old_session = self.race_state()['session']
        owned = self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
        self.stop_case('b3_vision_kill_latches_fault',
                       lambda: self.children.send('controller', signal.SIGKILL, vision=owned),
                       {'autonomy': lambda v: v['state'] == 'SENSOR_STOP' and bool(v['vision']['fault']) and
                        v['race']['latch'] == 'fault:vision', 'drive': lambda v: v == 0., 'safe': lambda v: v == 0.})
        self.stop_case('b3_vision_fault_ack_rejected',
                       lambda: self.expect(self.race_ack(), False, 'fixed_fault_present'),
                       {'autonomy': lambda v: bool(v['vision']['fault'])}, no_resume=True)
        def restart():
            self.children.stop('controller')
            self.color, self.images, self.held_stamp = 'green', True, None
            self.children.start('controller')
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline, lambda v: v.get('race') and
                      v['race']['session'] != old_session and v['vision']['stats']['accepted'] >= 3),
                      'restarted controller publishes a new session')
            self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
            lights_until(lambda v: latched('process_start')(v) and v['race']['visual_start_fact'],
                         'restart lights stay latched')
            self.expect(self.race_ack(session=old_session), False, 'session_mismatch')
            self.expect(self.race_ack(), True, 'policy_latch_cleared')
            baseline = self.monitor.sequence
            self.wait(lambda: self.fresh('autonomy', baseline, lambda v: v['race']['started'] and rearm_wait(v)),
                      'restart does not reuse previous ARMED')
        self.stop_case('b3_restart_requires_ack_and_new_arm_not_previous_armed', restart,
                       {'autonomy': lambda v: rearm_wait(v) and v['race']['session'] != old_session and
                        v['race']['intervention'] == 1, 'mcu': lambda v: v['state'] == 'ARMED'}, no_resume=True)
        self.call('disarm')
        self.call('arm')
        self.moving('b3_restart_resume_after_ack_rearm_and_lights')
        self.final_stop()

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

    def stop_after_failure(self):
        """진단 실패는 보존하고 정리 전에 별도 STOP을 시도한다. 실패 판정은 불변."""
        outcome = self.report['failure_stop'] = dict(attempted=True, acknowledged=False,
                                                     fresh_zero_and_model_stopped=False)
        # 이미 실패한 no-resume 감시가 STOP 전달 자체를 차단하지 않게 한다.
        # 감시의 기존 증거/실패 사례는 지우거나 통과로 바꾸지 않는다.
        self.monitor.zero_watch = None
        baseline = self.monitor.sequence
        try:
            self.call('stop')
            outcome['acknowledged'] = True
            self.wait(lambda: self.stopped(baseline), 'failure STOP fresh zero', timeout=5.)
            outcome['fresh_zero_and_model_stopped'] = True
        except Exception as error:
            outcome['error'] = repr(error)

    def snapshot_sources(self):
        names = ['scripts/validate_pipeline_faults_ros.py', *SOURCES, *FIXTURE_SOURCES, *PROVENANCE]
        self.report['source_snapshot_scope'] = dict(runtime_modules=SOURCES,
            validator='scripts/validate_pipeline_faults_ros.py', fixture_helpers=FIXTURE_SOURCES, provenance_only=PROVENANCE,
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

    def write_diagnostics(self):
        """재시작 구간 기록과 자식 진단 줄 요약. 저장 실패는 보고서에 남기고 판정은 바꾸지 않는다."""
        try:
            self.end_trace('interrupted_before_trace_end')
            summary = self.report['probe_log_summary'] = {}
            contexts = self.report['mcu_fault_contexts'] = []
            slow = self.report['child_slow_publishes'] = []
            for owned in self.children.children:
                record = owned['record']
                counts, rows = probe_log_rows((self.output / record['log']).read_text(
                    errors='replace').splitlines())
                summary[record['log']] = counts
                contexts.extend(dict(log=record['log'], **row) for row in rows['MCU_FAULT_CONTEXT='])
                slow.extend(dict(log=record['log'], role=record['role'], **row) for row in rows['SLOW_PUBLISH='])
            if self.traces:
                (self.output / 'restart_traces.json').write_text(json.dumps(self.traces, ensure_ascii=False,
                    allow_nan=False), encoding='utf-8')
                self.report['restart_trace_file'] = 'restart_traces.json'
                self.report['restart_trace_summary'] = [dict(name=t['name'], outcome=t['outcome'],
                    start_sim_s=t['start_sim_s'], end_sim_s=t['end_sim_s'], rows=len(t['rows']),
                    rows_complete=t['rows_complete'], ticks=len(t['ticks']),
                    process_samples=len(t['process_samples'])) for t in self.traces]
        except Exception as error:
            self.report['diagnostic_write_error'] = repr(error)

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
    parser.add_argument('--scenario', choices=('full', 'restart_repro', 'start_contract', 'race_session'), default='full',
                        help='restart_repro: ros_v2 failure segment only; not the 34-condition validation')
    parser.add_argument('--cycles', type=int, default=1, choices=(1, 2, 3))
    parser.add_argument('--signal-mode', choices=('temporary_3light', 'official_4red_candidate'), default='temporary_3light')
    parser.add_argument('--controller-profile', choices=('r0', 'g1_per_ray_candidate', 'g1_recent_observation_candidate'), default='r0')
    parser.add_argument('--recent-check-mode',choices=('continuity','evidence_expiry'),default='continuity',
                        help='evidence_expiry checks point support and later expiry, not sustained missing-ray progress')
    args = parser.parse_args()
    if (args.scenario in ('start_contract', 'race_session')) != (args.signal_mode == 'official_4red_candidate'):
        parser.error('official candidate is limited to start_contract/race_session scenarios')
    if args.controller_profile != 'r0' and args.scenario != 'full':
        parser.error('G1 candidate fault regression uses the full scenario only')
    if args.recent_check_mode!='continuity' and args.controller_profile!='g1_recent_observation_candidate':
        parser.error('recent evidence check requires recent observation profile')
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
    shm_before = shm_listing()
    import rclpy
    rclpy.init()
    validator = None
    try:
        validator = PipelineValidator(output, args.wall_cap, args.sleep_wall, args.scenario, args.cycles,
                                      args.signal_mode, args.controller_profile)
        validator.report['recent_check_mode']=args.recent_check_mode
        validator.report['dev_shm_before'] = shm_before
        validator.report['recent_continuous_progress_verified']=False
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
            validator.stop_after_failure()
            # 실패 회수 STOP 수신까지 같은 진단 창에 남긴다. 판정은 바꾸지 않는다.
            validator.end_trace('failed: ' + repr(error))
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
            validator.report['dev_shm_after_cleanup'] = shm_listing()
            validator.write_diagnostics()
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
