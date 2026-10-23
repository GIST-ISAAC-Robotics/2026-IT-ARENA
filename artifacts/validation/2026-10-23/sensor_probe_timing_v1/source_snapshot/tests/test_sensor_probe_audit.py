"""감사기의 수신 행/CDR 대응 반례. MCAP/ROS 읽기는 별도 실제 ROS smoke에서 확인한다."""
import hashlib
import json
import sys
from types import SimpleNamespace

import pytest

from arena_vehicle_interface.bag_contract import update_digest
from audit_sensor_probe import audit, strict_json
from sensor_recording_profile import PROBE_SCHEMA, select_profile, profile_metadata


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    topic = '/imu/data'
    records = [(topic, b'a', 1000), (topic, b'b', 2000)]
    rows = [dict(topic=topic, index=i+1, serialized_bytes=1, source_ns=i*100,
                 receive_ros_ns=(i+1)*1000, receive_monotonic_ns=(i+1)*10000,
                 writer_monotonic_ns=(i+1)*10000+100) for i in range(2)]
    class Reader:
        def __init__(self):
            self.pos = 0
        def has_next(self):
            return self.pos < len(records)
        def read_next(self):
            result = records[self.pos]
            self.pos += 1
            return result
    import audit_sensor_bag
    monkeypatch.setattr(audit_sensor_bag, 'reader_for', lambda *a: Reader())
    def deserialize(data, _type):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0 if data == b'a' else 100)))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', SimpleNamespace(deserialize_message=deserialize))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', SimpleNamespace(get_message=lambda name: object))
    (tmp_path/'bag').mkdir()
    (tmp_path/'bag/data.mcap').write_bytes(b'reader stub, not an MCAP')
    (tmp_path/'bag/metadata.yaml').write_text('stub')
    (tmp_path/'parameters.json').write_text('{}')
    p = select_profile('sensor_probe', 'system', [topic], 'drive_motor_shaft')
    d = hashlib.sha256()
    for _, data, stamp in records:
        update_digest(d, data, stamp)
    manifest = dict(schema=PROBE_SCHEMA, complete=True, drive_feedback_mode='drive_motor_shaft',
                    **profile_metadata(p), topics=p['topics'], raw_bytes=2,
                    stats={topic: dict(count=2, cdr_sha256=d.hexdigest())})
    def save(trailing=''):
        (tmp_path/'receipt.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows)+trailing)
        manifest['files_sha256'] = {f.relative_to(tmp_path).as_posix(): hashlib.sha256(f.read_bytes()).hexdigest()
                                    for f in tmp_path.rglob('*') if f.is_file() and f.name != 'manifest.json'}
        (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    save()
    return tmp_path, rows, records, manifest, save


def test_exact_receipt_cdr_correspondence_without_sensor_time_approval(fixture):
    root, _, _, _, _ = fixture
    result = audit(root)
    assert result['record_integrity_verified'] and result['counts'] == {'/imu/data': 2}
    assert result['source_stamp_zero'] == {'/imu/data': 1}
    assert not result['sensor_acquisition_time_verified']
    assert not result['controller_replay_eligible'] and not result['ready_to_drive']


@pytest.mark.parametrize('key,value', [
    ('index', True), ('index', 2), ('serialized_bytes', True), ('serialized_bytes', 4),
    ('receive_ros_ns', 1.0), ('receive_ros_ns', 1001), ('receive_monotonic_ns', -1),
    ('writer_monotonic_ns', 9999), ('source_ns', 1), ('source_ns', False),
    ('topic', '/drive'), ('source_ns', None),
])
def test_recomputed_file_hash_does_not_hide_semantic_corruption(fixture, key, value):
    root, rows, _, _, save = fixture
    rows[0][key] = value
    save()
    with pytest.raises(ValueError):
        audit(root)


@pytest.mark.parametrize('mutation', ['missing_row', 'extra_row', 'backwards_mono', 'backwards_bag',
                                     'wrong_count', 'wrong_digest', 'wrong_bytes', 'wrong_topic', 'oversize'])
def test_cross_file_integrity_semantics(fixture, mutation):
    root, rows, records, manifest, save = fixture
    if mutation == 'missing_row':
        rows.pop()
    elif mutation == 'extra_row':
        rows.append(rows[-1])
    elif mutation == 'backwards_mono':
        rows[1]['receive_monotonic_ns'] = 0
    elif mutation == 'backwards_bag':
        topic, data, _ = records[1]
        records[1] = (topic, data, 500)
        rows[1]['receive_ros_ns'] = 500
    elif mutation == 'wrong_count':
        manifest['stats']['/imu/data']['count'] = 1
    elif mutation == 'wrong_digest':
        manifest['stats']['/imu/data']['cdr_sha256'] = '0'*64
    elif mutation == 'wrong_bytes':
        manifest['raw_bytes'] = 3
    elif mutation == 'wrong_topic':
        records[0] = ('/scan', b'a', 1000)
    elif mutation == 'oversize':
        rows[0]['extra'] = 'x'*65536
    save()
    with pytest.raises(ValueError):
        audit(root)


@pytest.mark.parametrize('data', ['{"a": 1, "a": 2}', '{"a": NaN}', '{"a": Infinity}', '{"a": -Infinity}'])
def test_ambiguous_json_rejected(data):
    with pytest.raises(ValueError):
        strict_json(data)
