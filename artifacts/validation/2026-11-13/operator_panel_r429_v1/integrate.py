"""검토한 stage만 원본 SHA를 대조해 공통 소스로 옮긴다."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None

rows = json.loads((A/'changes.json').read_bytes())
assert not (A/'integration.json').exists()
for row in rows:
    path = row['path']
    assert (ROOT/path).resolve().is_relative_to(ROOT.resolve())
    assert digest(ROOT/path) == row['original_sha256'], path
    assert digest(A/'stage'/path) == row['staged_sha256'], path
for row in rows:
    destination = ROOT/row['path']
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes((A/'stage'/row['path']).read_bytes())
    assert digest(destination) == row['staged_sha256']
with (A/'integration.json').open('x', encoding='utf8') as stream:
    json.dump({'integrated': True, 'files': rows, 'root_review': {
        'default_b3_speed_cap_preserves_previous_maximum': True,
        'manual_mode_does_not_publish_in_autonomous_mode': True,
        'drive_still_passes_independent_guard': True,
        'manual_child_stop_and_reap': True,
        'actual_ros_http_fake_mcu_result': 'bundle/ros_it-arena-r429-panel-c/result_v2/result.json',
        'original_failure_retained': 'bundle/ros_it-arena-r429-panel-b',
        'physical_actuation_tested': False,
        'browser_visual_reviewed': False}}, stream, indent=2)
print(json.dumps({'integrated': True, 'files': len(rows)}))
