"""진단 원문에 FIFO 후보를 적용한다. ROS/장치/운영 설정은 사용하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct

from fifo_candidate import FifoAssociator


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def run(path):
    before = digest(path)
    parser = FifoAssociator()
    counts = Counter()
    previous = {}
    reversals = 0
    preserved = True
    with path.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            sample = parser.feed(row)
            if sample is None:
                continue
            key = sample.epoch, sample.kind
            if key in previous and sample.sensor_tick < previous[key]:
                reversals += 1
            previous[key] = sample.sensor_tick
            preserved &= sample.raw_xyz == struct.unpack_from('<hhh', bytes.fromhex(row['payload_hex']))
            if row['phase'] == 'measure':
                counts[sample.kind] += 1
    return dict(measured_samples=dict(counts), raw_values_preserved=preserved,
                stream_nondecreasing_errors=reversals, all_phase_stats=dict(parser.stats),
                input_sha256=before, input_unchanged=before == digest(path),
                source_sha256=digest(Path(__file__).with_name('fifo_candidate.py')),
                replay_source_sha256=digest(Path(__file__)),
                time_contract_verified=False, ros_connected=False, hardware_changed=False)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    assert not args.output.exists()
    result = run(args.input)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result))
