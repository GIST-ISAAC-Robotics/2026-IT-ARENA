"""A2 기록/재생 허용 목록과 ROS 독립 무결성 검사. 명령은 재생하지 않는다."""
import hashlib
import json
from pathlib import Path
import struct
from bisect import bisect_right
import math
from collections.abc import Mapping

INPUTS = {
    '/camera/color/image_raw': 'sensor_msgs/msg/Image',
    '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/wheel_states': 'sensor_msgs/msg/JointState',
}
OBSERVATIONS = {
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/drive/safe': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/autonomy/status': 'std_msgs/msg/String',
    '/safety/status': 'std_msgs/msg/String',
    '/actuation/status': 'std_msgs/msg/String',
    '/actuation/output_status': 'std_msgs/msg/String',
    '/diagnostics/timing': 'std_msgs/msg/String',
}
TOPICS = {**INPUTS, **OBSERVATIONS}
REQUIRED = set(INPUTS) | {'/drive', '/drive/safe', '/autonomy/status', '/safety/status', '/diagnostics/timing'}


def dds_metadata(info):
    """배포판별 사전/속성형 부가정보를 수용하며 없는 값은 0이 아니라 None으로 보존한다."""
    get = info.get if isinstance(info, Mapping) else lambda key: getattr(info, key, None)
    result = {key: get(key) for key in ('source_timestamp', 'received_timestamp',
              'publication_sequence_number', 'reception_sequence_number')}
    gid = get('publisher_gid')
    result['publisher_gid'] = bytes(gid).hex() if gid is not None else None
    return result


def replay_topic(topic):
    if topic not in INPUTS:
        raise ValueError('Only allowlisted sensors may be replayed: ' + topic)
    return '/replay' + topic


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


def load_manifest(root):
    root = Path(root).resolve()
    manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema') != 1 or manifest.get('complete') is not True:
        raise ValueError('Incomplete or unsupported recording')
    if manifest.get('queue_drops') or manifest.get('error') or manifest.get('clock_regressions'):
        raise ValueError('Recording contains known loss/error/clock regression')
    if manifest.get('topics') != TOPICS:
        raise ValueError('Topic contract mismatch')
    if not REQUIRED.issubset({t for t, s in manifest['stats'].items() if s['count'] > 0}):
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
