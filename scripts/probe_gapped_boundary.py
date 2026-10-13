#!/usr/bin/env python3
"""평가 전용: 긴 반대 벽 경로를 실제로 분리 관측된 선호 벽 조각으로 대조한다.

조각 사이의 구간을 연결하거나 자유 공간으로 채우지 않는다. 경계 증거일 뿐
주행 허가가 아니며 LocalFollow·독립 보호·경계 기억을 변경하지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.follow_trace import replay_events, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene

ROOT = Path(__file__).resolve().parents[1]


from arena_autonomy.boundary_evidence import fragment_evidence


def probe(events):
    baseline = replay_events(events)
    if not baseline['passed']:
        raise ValueError('current baseline replay mismatch')
    header = events[0]['data']
    config, motion = FollowConfig(**header['follow_config']), MotionConfig(**header['motion_config'])
    last_source, rows = None, []
    for event in events:
        if event['kind'] == 'reset':
            last_source = None
        if event['kind'] != 'call':
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        source = (data['epoch'], scan.header.stamp.sec, scan.header.stamp.nanosec)
        if source == last_source:
            continue
        last_source = source
        base = dict(seq=event['seq'], reference_s=data['reference_s'],
                    original_reason=event['data']['output'][2]['reason'],
                    original_speed=event['data']['output'][0])
        try:
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
            other_side = 'right' if data['side'] == 'left' else 'left'
            reference, info = local_path(points, other_side, config.wall_offset_m)
            eligible = (reference is not None and info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m
                        and abs(info['wall_distance_m']-config.wall_offset_m) <= config.boundary_anchor_error_m)
            evidence = fragment_evidence(points, reference, data['side'], config) if eligible else dict(
                supported=False, reason='opposite_reference_insufficient')
            rows.append({**base, 'reference_details': info, 'evidence': evidence})
        except MotionUnavailable as error:
            rows.append({**base, 'evidence': dict(supported=False, reason=str(error))})
    return dict(scope='fixed_input_boundary_evidence_only_not_free_space_or_driving',
                baseline_matched=baseline['matched'], calls=baseline['calls'], fresh_scenes=len(rows),
                reasons=dict(Counter(r['evidence']['reason'] for r in rows)),
                boundary_hold_supported=[r['reference_s'] for r in rows if r['evidence']['supported'] and
                                         r['original_reason'] == 'follow_boundary_unconfirmed'], rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    if not target.is_relative_to(ROOT/'artifacts') or target == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    target.mkdir(parents=True, exist_ok=False)
    paths = [ROOT/'scripts/probe_gapped_boundary.py', ROOT/'tests/test_gapped_boundary_evidence.py',
             *(ROOT/'src/arena_autonomy/arena_autonomy'/name for name in
               ('follow_trace.py', 'local_follow.py', 'local_path.py', 'scene_motion.py', 'local_scene.py', 'lidar_motion.py', 'boundary_evidence.py'))]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(target/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            bundle.write(path, path.relative_to(ROOT))
    before = hashlib.sha256(args.input.read_bytes()).hexdigest()
    with args.input.open(encoding='utf-8') as stream:
        result = probe([json.loads(line) for line in stream])
    result.update(input_sha256=before, input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  source_hashes=hashes, source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h
                                                         for p, h in hashes.items()))
    (target/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'source_hashes')}, ensure_ascii=False))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
