#!/usr/bin/env python3
"""저장된 정지 커브 스캔에 합성 오차를 더한 경계 사례 감사. 폐루프/실물 아님."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import types
import zipfile

import numpy as np
from arena_autonomy.local_path import pursuit_command

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts'):
        raise ValueError('output must be in artifacts')
    out.mkdir(parents=True, exist_ok=False)
    reference = ROOT/'artifacts/validation/2026-09-22/lidar_robustness/probe_v7/source_snapshot.zip'
    old = types.ModuleType('preserved_v7')
    # 저장소 자체의 이전 실행 소스만 읽는다. 외부 입력 코드를 실행하는 인터페이스가 아니다.
    with zipfile.ZipFile(reference) as archive:
        exec(archive.read('src/arena_autonomy/arena_autonomy/local_path.py'), old.__dict__)
    sources, cases = [], []
    for name in ('lap_configured_video_v2', 'lap_configured_v3', 'lap_configured_v5', 'lap_configured_v6_repeat2'):
        source = ROOT/'artifacts/validation/2026-09-21/local_pursuit'/name/'last_scan.json'
        sources.append(source)
        scan = json.loads(source.read_text())
        ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
        angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
        for version, function in (('v7', old.pursuit_command), ('v8', pursuit_command)):
            trials = []
            for seed in range(50):
                noisy = ranges+np.random.default_rng(seed).uniform(-.03, .03, len(ranges))
                valid = keep & (noisy >= scan['range_min']) & (noisy <= scan['range_max'])
                points = np.column_stack((noisy[valid]*np.cos(angles[valid])+.06,
                                          noisy[valid]*np.sin(angles[valid])))
                speed, steer, details = function(points, 'left', 0.)
                trials.append({'seed': seed, 'speed_mps': speed, 'steering_rad': steer,
                               'reason': details['reason'], 'fallback': details.get('wall_fallback')})
            cases.append({'case': name, 'version': version, 'stops': sum(t['speed_mps'] == 0 for t in trials),
                          'reasons': dict(Counter(t['reason'] for t in trials)), 'trials': trials})
    sources.extend([Path(__file__).resolve(), ROOT/'src/arena_autonomy/arena_autonomy/local_path.py'])
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(out/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for source in sources:
            archive.write(source, str(source.relative_to(ROOT)))
        archive.write(reference, 'preserved_v7_source_snapshot.zip')
    report = {'scope': 'static archived scans with synthetic radial error, not trajectory recovery or collision assessment',
              'range_error': 'independent uniform -0.03..0.03m', 'input_sha256': hashes, 'cases': cases}
    (out/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps([{k: v for k, v in c.items() if k != 'trials'} for c in cases], indent=2))


if __name__ == '__main__':
    main()
