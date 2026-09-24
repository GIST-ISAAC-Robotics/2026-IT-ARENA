#!/usr/bin/env python3
"""직선 벽의 극좌표 거리 오차에 대한 경로 생성 진단. 폐루프/실물 시험 아님."""
import argparse
import hashlib
import json
from pathlib import Path
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
    angles = np.linspace(-np.pi, np.pi, 500, endpoint=False)
    ranges = .425 / np.maximum(1e-9, np.abs(np.sin(angles)))
    valid = (ranges <= 12.) & (np.abs(angles) <= np.deg2rad(150))
    cases = [('clean', 0., 0., 0.), ('uniform30', .03, 0., 0.),
             ('bias_plus30', 0., 0., .03), ('bias_minus30', 0., 0., -.03),
             ('gaussian30_stress', 0., .03, 0.)]
    results = []
    for name, uniform, sigma, bias in cases:
        trials = []
        for seed in range(50):
            rng = np.random.default_rng(seed)
            r = ranges.copy() + rng.uniform(-uniform, uniform, 500) + rng.normal(0., sigma, 500) + bias
            points = np.column_stack((r[valid]*np.cos(angles[valid])+.06,
                                      r[valid]*np.sin(angles[valid])))
            speed, steer, meta = pursuit_command(points, 'left', 1.4, max_speed=2.5)
            trials.append({'seed': seed, 'speed_mps': speed, 'steering_rad': steer,
                           'reason': meta['reason'], 'fallback': meta.get('wall_fallback'),
                           'path_length_m': meta.get('observed_path_ahead_m')})
        results.append({'case': name, 'stopped': sum(t['speed_mps'] == 0 for t in trials),
                        'steering_abs_p95_rad': float(np.percentile([abs(t['steering_rad']) for t in trials],95)),
                        'trials': trials})
    sources = ['scripts/probe_local_path_noise.py', 'src/arena_autonomy/arena_autonomy/local_path.py']
    hashes = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
    with zipfile.ZipFile(out/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    (out/'report.json').write_text(json.dumps({'scope':'stationary synthetic straight wall, not closed loop',
        'input_sha256': hashes, 'results': results}, indent=2))
    print(json.dumps([{k:v for k,v in row.items() if k != 'trials'} for row in results], indent=2))


if __name__ == '__main__':
    main()
