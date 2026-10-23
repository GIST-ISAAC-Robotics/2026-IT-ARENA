#!/usr/bin/env python3
"""센서 점검용 CDR/수신 기록 감사. 파일만 읽으며 제어기 재생·시간 승인은 하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import sha256_file, update_digest
from sensor_recording_profile import PROBE_SCHEMA, profile_metadata, select_profile


def strict_json(data):
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise ValueError('duplicate JSON key')
            out[key] = value
        return out
    def invalid(value):
        raise ValueError('nonfinite JSON value: '+value)
    return json.loads(data, object_pairs_hook=pairs, parse_constant=invalid)


def read_json(path, max_bytes=4*1024*1024):
    with Path(path).open('rb') as stream:
        data = stream.read(max_bytes+1)
    if len(data) > max_bytes:
        raise ValueError('bounded JSON required')
    return strict_json(data)


def load_probe(root):
    root = Path(root).resolve()
    m = read_json(root/'manifest.json')
    if m.get('schema') != PROBE_SCHEMA or m.get('complete') is not True:
        raise ValueError('incomplete or unsupported sensor probe')
    if type(m.get('topics')) is not dict:
        raise ValueError('topic map required')
    p = select_profile(m.get('recording_profile'), m.get('recorder_clock_mode'),
                       list(m['topics']), m.get('drive_feedback_mode'))
    if p['name'] != 'sensor_probe' or m['topics'] != p['topics']:
        raise ValueError('sensor probe topic contract differs')
    # bool과 int의 느슨한 동등 비교로 잘못된 메타데이터를 통과시키지 않는다.
    for k, v in profile_metadata(p).items():
        if type(m.get(k)) is not type(v) or m.get(k) != v:
            raise ValueError('sensor probe metadata differs: '+k)
    if m.get('queue_drops') or m.get('error') or m.get('clock_regressions'):
        raise ValueError('sensor probe contains known loss/error/clock regression')
    if type(m.get('stats')) is not dict or set(m['stats']) != set(p['topics']):
        raise ValueError('sensor stats differ')
    for s in m['stats'].values():
        if type(s.get('count')) is not int or not 1 <= s['count'] <= 10_000_000:
            raise ValueError('bounded positive sample count required')
    files = m.get('files_sha256')
    if type(files) is not dict or not 3 <= len(files) <= 256:
        raise ValueError('bounded file inventory required')
    if not {'receipt.jsonl', 'parameters.json', 'bag/metadata.yaml'} <= set(files):
        raise ValueError('incomplete file inventory')
    mcap = set()
    for rel, digest in files.items():
        if (type(rel) is not str or '\\' in rel or PurePosixPath(rel).is_absolute()
                or '..' in PurePosixPath(rel).parts or ':' in rel
                or type(digest) is not str or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('invalid path/hash in inventory')
        path = (root/rel).resolve()
        if not path.is_relative_to(root) or not path.is_file() or sha256_file(path) != digest:
            raise ValueError('sensor probe file integrity failure: '+rel)
        if rel.endswith('.mcap'):
            mcap.add(rel)
    if not mcap or mcap != {p.relative_to(root).as_posix() for p in (root/'bag').glob('*.mcap')}:
        raise ValueError('MCAP inventory differs')
    if read_json(root/'parameters.json') != {}:
        raise ValueError('sensor-only capture must not collect controller parameters')
    return m


def audit(root):
    # Imports/reader are local: the metadata validator and tests need no ROS installation.
    from audit_sensor_bag import reader_for
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    root = Path(root).resolve()
    manifest = load_probe(root)
    topics = manifest['topics']
    types = {t: get_message(name) for t, name in topics.items()}
    reader_info = {}
    reader = reader_for(root, reader_info, topics)
    counts, digests = Counter(), {t: hashlib.sha256() for t in topics}
    previous_ros = previous_mono = None
    source_previous, source_regressions, source_zero = {}, Counter(), Counter()
    raw_bytes = 0
    with (root/'receipt.jsonl').open('rb') as receipt:
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            line = receipt.readline(65537)
            if not line or len(line) > 65536:
                raise ValueError('missing/oversized receipt row')
            row = strict_json(line)
            if type(row) is not dict:
                raise ValueError('receipt object required')
            for key in ('index', 'serialized_bytes', 'receive_ros_ns',
                        'receive_monotonic_ns', 'writer_monotonic_ns'):
                if type(row.get(key)) is not int or row[key] < 0:
                    raise ValueError('invalid receipt integer: '+key)
            if type(row.get('source_ns')) is not int:
                raise ValueError('original integer source stamp required')
            counts[topic] += 1
            if (topic not in topics or row['topic'] != topic or row['index'] != counts[topic]
                    or row['receive_ros_ns'] != stamp or row['serialized_bytes'] != len(data)):
                raise ValueError('CDR and receipt correspondence differs')
            if ((previous_ros is not None and stamp < previous_ros)
                    or (previous_mono is not None and row['receive_monotonic_ns'] < previous_mono)
                    or row['writer_monotonic_ns'] < row['receive_monotonic_ns']):
                raise ValueError('receipt clock ordering invalid')
            previous_ros, previous_mono = stamp, row['receive_monotonic_ns']
            message = deserialize_message(data, types[topic])
            source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if row['source_ns'] != source:
                raise ValueError('original source stamp differs')
            source_zero[topic] += source == 0
            if topic in source_previous:
                source_regressions[topic] += source < source_previous[topic]
            source_previous[topic] = source
            raw_bytes += len(data)
            update_digest(digests[topic], data, stamp)
        if receipt.read(1):
            raise ValueError('extra receipt data')
    for topic in topics:
        s = manifest['stats'][topic]
        if counts[topic] != s['count'] or digests[topic].hexdigest() != s['cdr_sha256']:
            raise ValueError('CDR count/digest differs: '+topic)
    if raw_bytes != manifest['raw_bytes']:
        raise ValueError('raw byte count differs')
    return dict(record_integrity_verified=True, profile='sensor_probe', counts=dict(counts),
                raw_bytes=raw_bytes, storage_reader=reader_info,
                recorder_clock_mode=manifest['recorder_clock_mode'],
                source_stamp_zero=dict(source_zero), source_stamp_regressions=dict(source_regressions),
                sensor_acquisition_time_verified=False, hardware_verified=False,
                upstream_loss_verified=False, controller_replay_eligible=False, ready_to_drive=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.recording)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result))
