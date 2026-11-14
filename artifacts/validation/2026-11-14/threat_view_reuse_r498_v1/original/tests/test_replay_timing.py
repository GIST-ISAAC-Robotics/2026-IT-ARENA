"""지연 저장 계측은 파일/순번 손상을 통과로 숨기지 않는다."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_timing import load_export


def fixture_export(tmp_path):
    path = tmp_path/'timing.jsonl'
    rows = [dict(node='test', callback='imu', sequence=i) for i in (1, 2)]
    payload = ''.join(json.dumps(row)+'\n' for row in rows).encode()
    path.write_bytes(payload)
    snapshot = dict(node='test', completed={'imu': 2}, timing_export=dict(
        mode='buffered', path=str(path), rows=2, dropped=0,
        sha256=hashlib.sha256(payload).hexdigest()))
    return path, snapshot, rows


def test_loads_exact_checkpoint(tmp_path):
    path, snapshot, rows = fixture_export(tmp_path)
    assert load_export(snapshot, path, 'test') == rows


@pytest.mark.parametrize('field,value', [('dropped', 1), ('rows', 3), ('mode', 'topic'),
    ('path', '/unexpected/file'), ('sha256', 'wrong')])
def test_rejects_wrong_metadata(tmp_path, field, value):
    path, snapshot, _ = fixture_export(tmp_path)
    snapshot['timing_export'][field] = value
    with pytest.raises(ValueError):
        load_export(snapshot, path, 'test')


@pytest.mark.parametrize('sequences,completed', [([1, 1], 2), ([1, 3], 3), ([1, 2], 1), ([0, 1], 1)])
def test_rejects_sequence_failure_even_with_correct_hash(tmp_path, sequences, completed):
    path, snapshot, _ = fixture_export(tmp_path)
    payload = ''.join(json.dumps(dict(node='test', callback='imu', sequence=i))+'\n'
                      for i in sequences).encode()
    path.write_bytes(payload)
    snapshot['timing_export']['sha256'] = hashlib.sha256(payload).hexdigest()
    snapshot['completed']['imu'] = completed
    with pytest.raises(ValueError):
        load_export(snapshot, path, 'test')
