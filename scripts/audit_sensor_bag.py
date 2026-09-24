#!/usr/bin/env python3
"""원시 CDR 무결성과 콜백 계측을 감사한다. ROS 발행 없이 파일만 읽는다."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import TOPICS, load_manifest, update_digest, quantiles


def reader_for(root):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    if {t.name: t.type for t in reader.get_all_topics_and_types()} != TOPICS:
        raise ValueError('MCAP topic/type contract differs from manifest')
    return reader


def summarize_timing(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row['node']+'/'+row['callback']].append(row)
    return {key: {metric: quantiles([r[metric] for r in values if r.get(metric) is not None])
                  for metric in ('duration_ms', 'source_age_ms', 'wall_interval_ms')}
            for key, values in sorted(groups.items())}


def audit(root):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from ackermann_msgs.msg import AckermannDriveStamped
    root = Path(root).resolve()
    manifest = load_manifest(root)
    reader = reader_for(root)
    counts = Counter()
    digests = {topic: hashlib.sha256() for topic in TOPICS}
    timing, commands = [], defaultdict(list)
    first = last = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if last is not None and stamp < last:
            raise ValueError('Bag time regresses')
        first = stamp if first is None else first
        last = stamp
        counts[topic] += 1
        update_digest(digests[topic], data, stamp)
        if topic == '/diagnostics/timing':
            timing.append(json.loads(deserialize_message(data, String).data))
        if topic in ('/drive', '/drive/safe'):
            message = deserialize_message(data, AckermannDriveStamped)
            commands[topic].append([stamp, message.drive.speed, message.drive.steering_angle])
    for topic in TOPICS:
        expected = manifest['stats'][topic]
        if counts[topic] != expected['count'] or digests[topic].hexdigest() != expected['cdr_sha256']:
            raise ValueError('CDR count/hash differs: '+topic)
    receipt_counts = Counter()
    age, write_wait = defaultdict(list), []
    previous = {}
    source_gaps, sequence_available = Counter(), Counter()
    with (root/'receipt.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            topic = row['topic']
            receipt_counts[topic] += 1
            if row['index'] != receipt_counts[topic]:
                raise ValueError('Receipt index mismatch')
            if row['source_ns'] is not None:
                age[topic].append((row['receive_ros_ns']-row['source_ns'])/1e6)
            write_wait.append((row['writer_monotonic_ns']-row['receive_monotonic_ns'])/1e6)
            sequence = row.get('publication_sequence_number')
            gid = row.get('publisher_gid')
            if sequence is not None and sequence > 0 and gid:
                sequence_available[topic] += 1
                key = (topic, gid)
                if key in previous and sequence > previous[key]+1:
                    source_gaps[topic] += sequence-previous[key]-1
                previous[key] = sequence
    if receipt_counts != counts:
        raise ValueError('Receipt/MCAP count mismatch')
    rates = {}
    for topic, stat in manifest['stats'].items():
        rates[topic] = {}
        for basis in ('source', 'receive'):
            first_stamp, last_stamp = stat['first_'+basis+'_ns'], stat['last_'+basis+'_ns']
            rates[topic][basis+'_sim_hz'] = ((stat['count']-1)*1e9/(last_stamp-first_stamp)
                if stat['count'] > 1 and first_stamp is not None and last_stamp > first_stamp else None)
    result = dict(verified=True, first_ns=first, last_ns=last, duration_sim_s=(last-first)/1e9,
        counts=dict(counts), raw_bytes=manifest['raw_bytes'],
        recorded_topic_rates=rates,
        source_age_ms={t: quantiles(v) for t, v in age.items()}, writer_wait_ms=quantiles(write_wait),
        source_sequence_gap_observations=dict(source_gaps), timing=summarize_timing(timing),
        source_sequence_available_samples=dict(sequence_available),
        source_gap_caveat='sequence counters may be unavailable; publisher restarts/multiple publishers are not inferred',
        commands=commands)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.recording)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('verified', 'duration_sim_s', 'counts', 'raw_bytes')}))
