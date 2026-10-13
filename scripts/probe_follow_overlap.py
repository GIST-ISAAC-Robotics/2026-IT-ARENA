#!/usr/bin/env python3
"""평가 전용: 경로 일치 검사를 실제 공통 관측 구간으로 한정한 후보 대조.

보존 입력의 command-stage 반사실 계산이다. 변경된 출력에 따른 미래 차체/
센서/직전 조향은 새로 생성하지 않으므로 폐루프 성능 판정으로 사용하지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import freeze, replay_call, replay_events
from arena_autonomy.lidar_motion import MotionConfig


def legacy_path_agrees(candidate, reference, tolerance):
    """10/13 수정 전 판정. 기존 기록 대조에만 쓰며 운영 경로에 연결하지 않는다."""
    paths = [np.asarray(p)[(np.asarray(p)[:, 0] >= .05) & (np.asarray(p)[:, 0] <= .65)]
             for p in (candidate, reference)]
    if any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return False
    distances = np.linalg.norm(paths[0][:, None, :]-paths[1][None, :, :], axis=2)
    return bool(max(np.percentile(np.min(distances, axis=0), 80),
                    np.percentile(np.min(distances, axis=1), 80)) <= tolerance)


# 운영 채택 전 후보의 소스는 analysis_source_before.zip에 보존했다.
shared_path_agrees = lf.path_agrees


def probe(events):
    with patch.object(lf, 'path_agrees', legacy_path_agrees):
        baseline = replay_events(events)
    if not baseline['passed']:
        raise ValueError('baseline replay mismatch')
    config = events[0]['data']
    follow = lf.LocalFollow(lf.FollowConfig(**config['follow_config']))
    motion = MotionConfig(**config['motion_config'])
    rows = []
    with patch.object(lf, 'path_agrees', shared_path_agrees):
        for event in events[1:]:
            if event['kind'] == 'reset':
                follow.reset()
            elif event['kind'] == 'call':
                data = event['data']
                candidate = freeze(replay_call(follow, data['input'], motion))
                old = data['output']
                rows.append({'seq': event['seq'], 't': data['input']['reference_s'],
                    'old_reason': old[2]['reason'], 'new_reason': candidate[2]['reason'],
                    'old_speed': old[0], 'new_speed': candidate[0],
                    'old_steering': old[1], 'new_steering': candidate[1],
                    'new_details': candidate[2]})
    return {'scope': 'fixed_input_counterfactual_not_closed_loop', 'baseline_matched': baseline['matched'],
            'old_reasons': dict(Counter(r['old_reason'] for r in rows)),
            'new_reasons': dict(Counter(r['new_reason'] for r in rows)),
            'release_proposals': [{'seq': r['seq'], 't': r['t'], 'new_speed': r['new_speed']} for r in rows
                                   if r['old_speed'] == 0 and r['new_speed'] > 0],
            'rows': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.input.read_bytes()).hexdigest()
    with args.input.open() as stream:
        events = [json.loads(line) for line in stream]
    result = probe(events)
    result.update(input_sha256=before, input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'release_proposals')}, ensure_ascii=False))
    print('release_proposals', len(result['release_proposals']))


if __name__ == '__main__':
    main()
