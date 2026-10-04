"""A2 기록/재생 허용 목록과 ROS 독립 무결성 검사. 명령은 재생하지 않는다.

schema 1은 구동 피드백 모드 필드가 없던 옛 기록이며 좌우 후륜 /wheel_states로만 읽는다.
schema 2는 drive_feedback_mode를 명시하고 그 모드의 피드백 토픽 하나만 허용한다.
재생은 기록된 제어기 매개변수(또는 옛 기록의 legacy 기본값)를 쓰며 현재 vehicle.yaml을
읽어 옛 입력을 새 반지름·감속비로 다시 환산하지 않는다.
"""
import hashlib
import json
from pathlib import Path
import struct
from bisect import bisect_right
import math
from collections.abc import Mapping

from arena_vehicle_interface.drive_feedback import MODE_LEGACY, MODE_MOTOR, MODES, TOPIC_BY_MODE

SENSOR_INPUTS = {
    '/camera/color/image_raw': 'sensor_msgs/msg/Image',
    '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/imu/data': 'sensor_msgs/msg/Imu',
}
LEGACY_DEFAULT_WHEEL_RADIUS_M = .025  # 모드 필드 이전 제어기 기본값. 현재 차량값이 아니다.
OBSERVATIONS = {
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/drive/safe': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/autonomy/status': 'std_msgs/msg/String',
    '/safety/status': 'std_msgs/msg/String',
    '/actuation/status': 'std_msgs/msg/String',
    '/actuation/output_status': 'std_msgs/msg/String',
    '/diagnostics/timing': 'std_msgs/msg/String',
}
REQUIRED_OBSERVATIONS = {'/drive', '/drive/safe', '/autonomy/status', '/safety/status', '/diagnostics/timing'}


def inputs_for(mode):
    if mode not in MODES:
        raise ValueError('Unknown drive feedback mode: ' + repr(mode))
    return {**SENSOR_INPUTS, TOPIC_BY_MODE[mode]: 'sensor_msgs/msg/JointState'}


def topics_for(mode):
    return {**inputs_for(mode), **OBSERVATIONS}


def required_for(mode):
    return set(inputs_for(mode)) | REQUIRED_OBSERVATIONS


# schema 1(legacy) 계약. 새 기록/재생은 manifest의 모드로 topics_for()를 사용한다.
INPUTS = inputs_for(MODE_LEGACY)
TOPICS = topics_for(MODE_LEGACY)
REQUIRED = required_for(MODE_LEGACY)


def manifest_feedback_mode(manifest):
    """schema 1은 legacy로 고정, schema 2는 명시 모드만 허용한다."""
    if manifest.get('schema') == 1:
        if 'drive_feedback_mode' in manifest:
            raise ValueError('Schema 1 recording cannot declare a drive feedback mode')
        return MODE_LEGACY
    if manifest.get('schema') == 2 and manifest.get('drive_feedback_mode') in MODES:
        return manifest['drive_feedback_mode']
    raise ValueError('Incomplete or unsupported recording')


def dds_metadata(info):
    """배포판별 사전/속성형 부가정보를 수용하며 없는 값은 0이 아니라 None으로 보존한다."""
    get = info.get if isinstance(info, Mapping) else lambda key: getattr(info, key, None)
    result = {key: get(key) for key in ('source_timestamp', 'received_timestamp',
              'publication_sequence_number', 'reception_sequence_number')}
    gid = get('publisher_gid')
    result['publisher_gid'] = bytes(gid).hex() if gid is not None else None
    return result


def replay_topic(topic, mode=MODE_LEGACY):
    if topic not in inputs_for(mode):
        raise ValueError('Only allowlisted sensors may be replayed: ' + topic)
    return '/replay' + topic


def resolve_replay_feedback(manifest, parameters):
    """기록 모드와 제어기 매개변수를 대조해 재생용 명시 매개변수를 만든다.

    현재 vehicle.yaml은 읽지 않는다. 옛 schema 1에 모드가 없으면 legacy, 반지름이 없으면
    당시 기본값 0.025 m를 명시한다. 모드가 서로 다르거나 모터축 기하가 없으면 거절한다.
    """
    mode = manifest_feedback_mode(manifest)
    resolved, geometry = {}, set()
    for name, values in parameters.items():
        declared = values.get('motion_feedback_mode')
        if declared is None and manifest['schema'] != 1:
            raise ValueError('Recorded controller lacks motion_feedback_mode: ' + name)
        if (declared or MODE_LEGACY) != mode:
            raise ValueError('Controller feedback mode differs from recording: ' + name)
        params = dict(values, motion_feedback_mode=mode)
        source = 'recorded_parameters'
        if 'motion_wheel_radius_m' not in params:
            if mode != MODE_LEGACY:
                raise ValueError('Recorded motor-shaft controller lacks wheel radius: ' + name)
            params['motion_wheel_radius_m'], source = LEGACY_DEFAULT_WHEEL_RADIUS_M, 'legacy_controller_default'
        radius = params['motion_wheel_radius_m']
        ratio = params.get('motion_gear_ratio') if mode == MODE_MOTOR else None
        # 모터축 모드는 감속비 없이는 평균 바퀴 속도를 만들 수 없다. 누락/None도 거절한다.
        for value in (radius, ratio) if mode == MODE_MOTOR else (radius,):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('Recorded drive feedback geometry invalid: ' + name)
        geometry.add((float(radius), None if ratio is None else float(ratio), source))
        resolved[name] = params
    if len(geometry) != 1:
        raise ValueError('Controllers disagree on recorded drive feedback geometry')
    radius, ratio, source = geometry.pop()
    return dict(mode=mode, topic=TOPIC_BY_MODE[mode], wheel_radius_m=radius, gear_ratio=ratio,
                geometry_source=source, current_vehicle_yaml_used=False), resolved


def update_digest(digest, payload, timestamp):
    digest.update(struct.pack('<QQ', timestamp, len(payload)))
    digest.update(payload)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def quantiles(values):
    values = sorted(values)
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'max': None}
    return dict(count=len(values), **{key: values[min(len(values)-1, int((len(values)-1)*q))]
                                    for key, q in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1.)]})


def eof_stopped(commands, last_input_ns, tail_ns=1_500_000_000):
    """입력 종료 직전의 0명령을 EOF 정지로 잘못 통과시키지 않는다."""
    tail = [row for row in commands if row[0] >= last_input_ns+tail_ns-300_000_000]
    return len(tail) >= 10 and all(math.isfinite(row[1]) and abs(row[1]) < 1e-8 for row in tail)


def compare_commands(original, replayed, last_input_ns):
    """비동기 타이머의 출력 비교용 지표. 시각별 이전 명령과 비교하며 합격 기준은 아니다."""
    stamps = [r[0] for r in original]
    speed, steering = [], []
    for stamp, velocity, angle in replayed:
        index = bisect_right(stamps, stamp)-1
        if index < 0 or stamp > last_input_ns:
            continue
        speed.append(abs(velocity-original[index][1]))
        steering.append(abs(angle-original[index][2]))
    return dict(samples=len(speed), speed_absolute_error_mps=quantiles(speed),
                steering_absolute_error_rad=quantiles(steering),
                scope='zero-order previous recorded command; asynchronous timing differences included; not trajectory error')


def active_window_continuity(original, replayed):
    """기준 기록의 이동 구간에서 새 안전 명령의 0/역행/비유한 값을 별도 판정한다."""
    moving = [r[0] for r in original if math.isfinite(r[1]) and r[1] > .1]
    if not moving:
        return dict(no_interruption=False, reason='no_reference_moving_window', samples=0)
    start, end = min(moving), max(moving)
    selected = [r for r in replayed if start <= r[0] <= end]
    invalid = sum(not math.isfinite(r[1]) or not math.isfinite(r[2]) for r in selected)
    zeros = sum(math.isfinite(r[1]) and abs(r[1]) < 1e-8 for r in selected)
    reverse = sum(math.isfinite(r[1]) and r[1] < -1e-8 for r in selected)
    return dict(no_interruption=bool(selected) and not (invalid or zeros or reverse),
        reference_start_ns=start, reference_end_ns=end, samples=len(selected),
        zero_commands=zeros, reverse_commands=reverse, invalid_commands=invalid,
        scope='Observed commands in the reference moving window; not continuous coverage or physical motion proof')


def load_manifest(root):
    root = Path(root).resolve()
    manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('complete') is not True:
        raise ValueError('Incomplete or unsupported recording')
    mode = manifest_feedback_mode(manifest)
    if manifest.get('queue_drops') or manifest.get('error') or manifest.get('clock_regressions'):
        raise ValueError('Recording contains known loss/error/clock regression')
    if manifest.get('topics') != topics_for(mode):
        raise ValueError('Topic contract mismatch')
    if not required_for(mode).issubset({t for t, s in manifest['stats'].items() if s['count'] > 0}):
        raise ValueError('Required topics missing')
    for relative, digest in manifest['files_sha256'].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file() or sha256_file(path) != digest:
            raise ValueError('Recording integrity failure: ' + relative)
    # 파일 목록에서 원시 데이터/수신 시각/매개변수 누락도 거부한다.
    required_files = {'receipt.jsonl', 'parameters.json', 'bag/metadata.yaml'}
    if not required_files.issubset(manifest['files_sha256']) or not any(k.endswith('.mcap') for k in manifest['files_sha256']):
        raise ValueError('Incomplete file inventory')
    return manifest
