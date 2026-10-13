#!/usr/bin/env python3
"""고정 입력에서 점별 취득 시각 후보만 비교. 물리/폐루프 주행 개선 증거가 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import zipfile

from arena_autonomy.follow_trace import replay_events, replay_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

ROOT = Path(__file__).resolve().parents[1]


def probe(events):
    baseline = replay_events(events)
    if not baseline['passed']:
        raise ValueError('baseline replay mismatch')
    header = events[0]['data']
    config = FollowConfig(**header['follow_config'])
    if config.occupancy_timing_mode != 'oldest':
        raise ValueError('oldest baseline required')
    config = replace(config, occupancy_timing_mode='per_ray_candidate')
    controller, motion = LocalFollow(config), MotionConfig(**header['motion_config'])
    rows = []
    for event in events[1:]:
        if event['kind'] == 'reset': controller.reset()
        if event['kind'] != 'call': continue
        data = event['data']
        proposed = freeze(replay_call(controller, data['input'], motion))
        original = data['output']
        rows.append(dict(seq=event['seq'], t=data['input']['reference_s'],
                         before_speed=original[0], after_speed=proposed[0],
                         before_reason=original[2]['reason'], after_reason=proposed[2]['reason'],
                         before_clearance=original[2].get('swept_clearance_m'),
                         after_clearance=proposed[2].get('swept_clearance_m'),
                         occupied_before=original[2].get('occupied_points'),
                         occupied_after=proposed[2].get('occupied_points'),
                         padding_points=proposed[2].get('occupancy_padding_points')))
    preserved = all(row['occupied_before'] == row['occupied_after'] for row in rows)
    return dict(scope='fixed_recorded_future_and_previous_steering_not_closed_loop',
        baseline_matched=baseline['matched'], calls=len(rows), all_occupied_points_preserved=preserved,
        before_reasons=dict(Counter(row['before_reason'] for row in rows)),
        after_reasons=dict(Counter(row['after_reason'] for row in rows)),
        changed_speed_calls=sum(abs(row['before_speed']-row['after_speed']) > 1e-10 for row in rows),
        driving_authorized=False, rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.relative_to(ROOT/'artifacts')
    output.mkdir(exist_ok=False, parents=True)
    paths = [Path(__file__), ROOT/'tests/test_occupancy_timing.py',
        *(ROOT/'src/arena_autonomy/arena_autonomy'/f'{name}.py' for name in (
            'occupancy_timing','local_follow','local_path','scene_motion','local_scene','lidar_motion','follow_trace'))]
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths: bundle.write(path, path.relative_to(ROOT))
    before = hashlib.sha256(args.input.read_bytes()).hexdigest()
    with args.input.open() as stream: result = probe([json.loads(line) for line in stream])
    result.update(input_sha256=before, input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest(),
        source_sha256=hashes, source_unchanged=all(hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == digest
                                                for path,digest in hashes.items()))
    (output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({key:value for key,value in result.items() if key not in ('rows','source_sha256')}))
    return 0 if result['input_unchanged'] and result['source_unchanged'] and result['all_occupied_points_preserved'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
