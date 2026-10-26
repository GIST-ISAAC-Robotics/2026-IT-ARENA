"""run32 정지 장면의 직진 노면 검사 범위만 분해한다. 제어/판정 변경 없음."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import arena_autonomy.m3_execution_trial as mx


def probe(source):
    header = event = None
    for line in source.open(encoding='utf8'):
        item = json.loads(line)
        if item['kind'] == 'header':
            header = item['data']
        elif item['kind'] == 'call' and item['data']['input']['reference_s'] >= 32.5:
            event = item['data']
            break
    assert header and event
    detail = event['output'][2]
    ctl = mx.M3ExecutionFollow(mx.FollowConfig(**header['follow_config']),
                              mx.ExecutionConfig(**header['execution_config']),
                              allow_simulation_trial=True)
    cfg = ctl.lateral_config
    frame = mx.ExecutionFrame(detail['path_points'])
    candidate = mx.straight_candidate(frame, mx.ego_state(frame, cfg), cfg)
    registered = detail['m3_lateral']['checks']['straight']
    full = mx.surface_check(frame, candidate, cfg, current_pose=(0., 0., 0.))
    assert full['ok'] == registered['surface']['ok']
    assert full['reason'] == registered['surface']['reason']
    rows = []
    for count in range(2, len(candidate['poses']) + 1):
        short = dict(candidate)
        for key in ('poses', 'steering_rad', 'progress'):
            short[key] = candidate[key][:count]
        check = mx.surface_check(frame, short, cfg, current_pose=(0., 0., 0.))
        rows.append({'end_progress_m': float(short['progress'][-1]),
                     'ok': check['ok'], 'reason': check['reason'],
                     'grass_side': check['grass_side'],
                     'body_excess_m': check['max_body_excess_m']})
    required = registered['required_progress_m']
    covering = next((row for row in rows if row['end_progress_m'] >= required), None)
    return {'scope': 'one saved sensor-derived path; surface-only prefix comparison; not a new drive or safe command',
            'reference_s': event['input']['reference_s'],
            'registered_surface': registered['surface'],
            'registered_required_progress_m': required,
            'registered_committed_progress_m': registered['committed_progress_m'],
            'full_candidate_length_m': float(candidate['progress'][-1]),
            'full_candidate_curved': candidate['curved'],
            'margin_kept_for_all_prefixes_m': full['edge_margin_m'],
            'first_failed_prefix': next((row for row in rows if not row['ok']), None),
            'prefix_covering_required_progress': covering,
            'prefixes': rows,
            'limitation': 'A shorter prefix passing surface_check does not establish observation, clearance, command acceptance, recoverability, or safe continued progress. No horizon or margin was changed.',
            'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}


if __name__ == '__main__':
    source, destination = map(Path, sys.argv[1:])
    if destination.exists():
        raise FileExistsError(destination)
    result = probe(source)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps({key: value for key, value in result.items() if key not in ('prefixes', 'registered_surface')}, ensure_ascii=False))
