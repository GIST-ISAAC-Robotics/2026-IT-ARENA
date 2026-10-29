"""실제 저장 입력의 두 시점으로 수치 정합 실패를 재현한다. 새 주행은 아니다."""
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT / 'src/arena_autonomy'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.partial_surface_alignment import surface_translation
from perturb import CASES, transform

trace = BASE.parent / 'official_traffic_v1/run2_course/follow_decisions.jsonl'
target = BASE / 'surface_convergence_case_v1.json'
if target.exists():
    raise FileExistsError(target)
with trace.open('rb') as stream:
    before = hashlib.file_digest(stream, 'sha256').hexdigest()
inputs = {}
with trace.open(encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'header':
            config = MotionConfig(**row['data']['motion_config'])
        if row['kind'] != 'call':
            continue
        time = row['data']['input']['reference_s']
        for wanted in (133.72, 133.82):
            if abs(time - wanted) < 1e-6:
                inputs[wanted] = row['data']['input']
        if len(inputs) == 2:
            break
case = next(c for c in CASES if c.name == 'angle')
projected = {}
for wanted, raw in inputs.items():
    data, scan, history = restore_call(transform(raw, case), config)
    time = data['reference_s']
    source = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
    points, ids, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment,
        scan.range_min, scan.range_max, source, scan.time_increment, time, 'both', True)
    projected[wanted] = (points, ids, history, time)
old, _, _, old_time = projected[133.72]
current, ids, history, now = projected[133.82]
carried = transport_points(old, history, old_time, now)
fit = surface_translation(carried, current, ids)
fit.pop('processing_ms', None)
assert fit['reason'] == 'unsettled_or_high_residual'
assert abs(fit['steps'][-1]['step_m'][0] - (-7.593590861145136e-05)) < 1e-10
with trace.open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == before
result = dict(scope='two recorded scan surfaces; no truth input or new driving',
    input_sha256=before, source_reference_s=old_time, target_reference_s=now,
    previous_now=carried.tolist(), current=current.tolist(), source_indices=ids.tolist(),
    original_fit=fit)
with target.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(dict(points=[len(old), len(current)], original_fit=fit), ensure_ascii=False))
