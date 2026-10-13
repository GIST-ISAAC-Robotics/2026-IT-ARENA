#!/usr/bin/env python3
"""가림 경계 후보의 고정 입력 반사실 계산. 미래 센서·조향 응답은 재계산하지 않는다."""
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

ROOT = Path(__file__).resolve().parents[1]


def probe(events):
    baseline = replay_events(events)
    if not baseline['passed']:
        raise ValueError('baseline replay mismatch')
    header = events[0]['data']
    candidate = LocalFollow(replace(FollowConfig(**header['follow_config']), boundary_evidence_mode='fragmented_candidate'))
    motion = MotionConfig(**header['motion_config'])
    rows = []
    for event in events[1:]:
        if event['kind'] == 'reset':
            candidate.reset()
        elif event['kind'] == 'call':
            data = event['data']
            result = freeze(replay_call(candidate, data['input'], motion))
            rows.append(dict(seq=event['seq'], reference_s=data['input']['reference_s'],
                             original=data['output'], candidate=result))
    return dict(scope='fixed_input_counterfactual_not_closed_loop', baseline_matched=baseline['matched'],
                calls=len(rows), reasons=dict(Counter(row['candidate'][2]['reason'] for row in rows)),
                positive_proposals=[row['reference_s'] for row in rows if row['candidate'][0] > 0 and row['original'][0] == 0],
                rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    if not target.is_relative_to(ROOT/'artifacts') or target == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    target.mkdir(parents=True, exist_ok=False)
    paths = [ROOT/'scripts/probe_fragmented_follow.py', ROOT/'tests/test_fragmented_follow.py',
             *(ROOT/'src/arena_autonomy/arena_autonomy'/name for name in
               ('follow_trace.py', 'local_follow.py', 'local_path.py', 'scene_motion.py', 'local_scene.py',
                'lidar_motion.py', 'boundary_evidence.py'))]
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
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'source_hashes', 'positive_proposals')}, ensure_ascii=False))
    print('positive_proposals', len(result['positive_proposals']))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
