#!/usr/bin/env python3
"""보존 센서 입력에서 급커브 후보를 대조한다. 새 궤적/폐루프 합격이 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import zipfile

from arena_autonomy.follow_trace import freeze, replay_call, replay_events
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from profile_follow_trace import SNAPSHOT_MODULES

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def counterfactual(events, mode='corner_anchor_candidate'):
    """기존 입력/이전 조향은 그대로 사용한다. 후보 출력으로 다음 입력을 만들지 않는다."""
    controller = motion = None
    rows = []
    for event in events:
        kind, data = event['kind'], event['data']
        if kind == 'header':
            motion = MotionConfig(**data['motion_config'])
            controller = LocalFollow(replace(FollowConfig(**data['follow_config']),
                                             boundary_evidence_mode=mode))
        elif kind == 'reset':
            controller.reset()
        elif kind == 'call':
            actual = freeze(replay_call(controller, data['input'], motion))
            before = data['output']
            rows.append(dict(seq=event['seq'], reference_s=data['input']['reference_s'],
                             original_speed_mps=before[0], candidate_speed_mps=actual[0],
                             original_steering_rad=before[1], candidate_steering_rad=actual[1],
                             original_reason=before[2]['reason'], candidate_detail=actual[2]))
    selections = Counter(row['candidate_detail'].get('boundary_selection', 'none') for row in rows)
    return dict(scope='fixed_sensor_input_counterfactual_not_closed_loop',
                closed_loop_pass=False, driving_authorized=False, calls=len(rows),
                selections=dict(selections),
                newly_positive=sum(row['original_speed_mps'] == 0 and row['candidate_speed_mps'] > 0
                                   for row in rows),
                newly_zero=sum(row['original_speed_mps'] > 0 and row['candidate_speed_mps'] == 0
                               for row in rows), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('corner_anchor_candidate', 'corner_distance_candidate'), default='corner_anchor_candidate')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.input)
    paths = [f'src/{package}/{package}/{module}.py' for package, modules in SNAPSHOT_MODULES.items()
             for module in modules]
    paths += ['scripts/probe_corner_follow.py', 'scripts/profile_follow_trace.py',
              'tests/test_corner_evidence.py', 'tests/test_corner_follow.py']
    hashes = {p: digest(ROOT/p) for p in paths}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(ROOT/p, p)
    with args.input.open(encoding='utf-8') as stream:
        original = replay_events(json.loads(line) for line in stream)
    (args.output/'original_replay.json').write_text(json.dumps(original, indent=2, allow_nan=False)+'\n',
                                                 encoding='utf-8')
    if not original['passed']:
        raise RuntimeError('original mode replay mismatch; candidate comparison not run')
    with args.input.open(encoding='utf-8') as stream:
        result = counterfactual((json.loads(line) for line in stream), args.mode)
    result.update(original_replay_matched=original['matched'], input_sha256=before,
                  input_unchanged=before == digest(args.input), source_sha256=hashes,
                  source_unchanged=all(digest(ROOT/p) == value for p, value in hashes.items()))
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256')}))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
