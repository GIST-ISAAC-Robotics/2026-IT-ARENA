"""run2 최초 기억과 새 스캔의 재관측 거절을 센서 입력만으로 분리한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, thaw, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.anchor_clearance_trial import PositiveClearanceTrial
from arena_autonomy.scene_motion import motion_scene


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def nearest_segments(point, current, ids):
    vectors = np.diff(current, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    frac = np.sum((point - current[:-1]) * vectors, axis=1) / np.maximum(lengths**2, 1e-12)
    distance = np.linalg.norm(point-current[:-1]-np.clip(frac, 0, 1)[:, None]*vectors, axis=1)
    valid = (np.diff(ids) == 1) & (lengths > 1e-6) & (lengths < .15)
    chosen = [int(i) for i in np.argsort(distance) if valid[i]][:4]
    return [dict(ids=ids[i:i+2].tolist(), length_m=float(lengths[i]),
        fraction=float(frac[i]), distance_m=float(distance[i]),
        points=current[i:i+2].tolist(), adjacent_lengths_m=lengths[max(0,i-1):i+2].tolist())
        for i in chosen]


def main():
    trace = BASE / 'run2_course/follow_decisions.jsonl'
    target = BASE / 'run2_reobservation_v1.json'
    fixture = BASE / 'run2_reobservation_fixture_v1.json'
    if target.exists() or fixture.exists():
        raise FileExistsError('preserved diagnostic exists')
    before = digest(trace)
    calls = []
    with trace.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'header':
                header = row['data']
            if row['kind'] == 'call' and 133.4 <= row['data']['input']['reference_s'] <= 134.7:
                calls.append(row)
    event = next(r for r in calls if abs(r['data']['input']['reference_s']-133.56) < 1e-7)
    evidence = thaw(event['data']['output'])[2]['partial_observation']['latch_evidence']
    context = evidence['correspondence_context']
    original = next(r for r in calls if abs(r['data']['input']['reference_s']-evidence['previous_reference_s']) < 1e-7)
    config = MotionConfig(**header['motion_config'])
    data, scan, history = restore_call(original['data']['input'], config)
    points, ids, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment,
        scan.range_min, scan.range_max, evidence['previous_source_first_s'], scan.time_increment,
        data['reference_s'], 'both', True)
    keep = np.linalg.norm(points, axis=1) <= 3.
    old, old_ids = points[keep], ids[keep]
    np.testing.assert_allclose(old, context['previous_points_at_reference_xy'], rtol=0, atol=1e-8)
    carried = np.asarray(context['previous_points_now_xy'])
    retained_indices = []
    for witness in evidence['witnesses']:
        distances = np.linalg.norm(carried-witness['previous_point_now_xy'], axis=1)
        index = int(np.argmin(distances))
        assert distances[index] < 1e-8
        retained_indices.append(index)
    retained_indices = sorted(set(retained_indices))
    assert len(retained_indices) == evidence['pairs'] == 2
    anchor, clearance = SurfaceAnchorTrial(), PositiveClearanceTrial()
    for probe in (anchor, clearance):
        probe.start(old[retained_indices], old, old_ids, evidence['previous_reference_s'], evidence['previous_source_first_s'])
    rows, inputs, last_source = [], [], None
    for event in calls:
        data, scan, history = restore_call(event['data']['input'], config)
        now = data['reference_s']
        if now < evidence['reference_s']-1e-8:
            continue
        source = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
        scene = motion_scene(scan, history, now, timing_verified=True, rear_half_angle_deg=0.)
        current, ids, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, source, scan.time_increment, now, 'both', True)
        a = anchor.observe(current, ids, history, now, source)
        c = clearance.observe(scene, history, current, ids)
        for result in (a, c):
            if isinstance(result.get('fit'), dict):
                result['fit'].pop('processing_ms', None)
        if source == last_source:
            continue
        last_source = source
        row = dict(t=now, source=source, anchor=a, clearance=c,
            recorded_partial=thaw(event['data']['output'])[2].get('partial_observation'))
        row['recorded_partial'].pop('latch_evidence', None)
        for key in ('surface_correspondence',):
            if isinstance(row['recorded_partial'].get(key), dict):
                row['recorded_partial'][key].pop('processing_ms', None)
        if a.get('corrected_points') is not None:
            row['nearest_segments'] = [nearest_segments(np.asarray(p), current, ids) for p in a['corrected_points']]
        rows.append(row)
        inputs.append(event['data']['input'])
    assert digest(trace) == before
    result = dict(scope='isolated original anchor probes, no full control replay or new driving',
        input_sha256=before, input_unchanged=True, initial_evidence=evidence,
        retained_indices=retained_indices, retained_original=old[retained_indices].tolist(), rows=rows)
    target.write_text(json.dumps(freeze(result), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    fixture.write_text(json.dumps(dict(motion_config=header['motion_config'],
        original_input=original['data']['input'], retained_indices=retained_indices, inputs=inputs),
        ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps([dict(t=r['t'], anchor=r['anchor']['reason'], clearance=r['clearance']['reason'],
        distance=r['anchor'].get('nearest_distances_m'), segments=r.get('nearest_segments')) for r in rows], ensure_ascii=False))


if __name__ == '__main__':
    main()
