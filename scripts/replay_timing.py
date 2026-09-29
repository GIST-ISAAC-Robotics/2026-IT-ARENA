"""EOF에서 저장한 계측의 경로·해시·완료 순번을 확인한다. 센서/제어 변경 없음."""
import hashlib
import json
from pathlib import Path

from replay_observer import sequence_coverage


def load_export(snapshot, expected_path, expected_node):
    meta = snapshot['timing_export']
    if snapshot['node'] != expected_node or meta['mode'] != 'buffered':
        raise ValueError('Timing export identity mismatch')
    path = Path(expected_path).resolve()
    if Path(meta['path']).resolve() != path or meta['dropped'] != 0:
        raise ValueError('Timing export path mismatch or buffer overflow')
    payload = path.read_bytes()
    if hashlib.sha256(payload).hexdigest() != meta['sha256']:
        raise ValueError('Timing export hash mismatch')
    rows = [json.loads(line) for line in payload.splitlines()]
    if len(rows) != meta['rows'] or any(r['node'] != expected_node for r in rows):
        raise ValueError('Timing export count or node mismatch')
    completed = {expected_node+'/'+key: value for key, value in snapshot['completed'].items()}
    if any(r['callback'] not in snapshot['completed'] for r in rows):
        raise ValueError('Unknown callback in timing export')
    coverage = sequence_coverage(rows, completed)
    if any(not entry['complete'] or entry['after_snapshot'] for entry in coverage.values()):
        raise ValueError('Timing export sequence coverage failed')
    return rows
