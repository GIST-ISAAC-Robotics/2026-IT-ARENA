#!/usr/bin/env python3
"""독립 합성 벽과 알려진 운동으로 보정 정확도·계수 민감도 검증. 주행 시험 아님."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src/arena_autonomy'))
from arena_autonomy.lidar_motion import MotionHistory, MODES


def oracle(t):
    # 외부 기대값: 유효 벽 광선 취득 중인 0.17 s에서 twist 변경, body 중앙 기준.
    x = y = yaw = 0.
    for duration, v, w in ((min(t, .17), 2., .5), (max(0., t-.17), 5.5, 2.)):
        a = w * duration
        dx, dy = v/w * math.sin(a), v/w * (1-math.cos(a))
        x, y = x + math.cos(yaw)*dx-math.sin(yaw)*dy, y + math.sin(yaw)*dx+math.cos(yaw)*dy
        yaw += a
    return np.array([x + .0725*math.cos(yaw), y + .0725*math.sin(yaw), yaw])


def world(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(REPO / 'artifacts'):
        raise ValueError('artifacts output required')
    output.mkdir(parents=True, exist_ok=False)
    stamp, dt, now = .1, .0002, .25
    angles = np.linspace(-math.pi, math.pi, 500)
    ranges = []
    for i, angle in enumerate(angles):
        p = oracle(stamp+i*dt)
        sy = p[1] - .03 * math.sin(p[2])
        denominator = math.sin(p[2]+angle)
        r = (2.-sy)/denominator if abs(denominator) > 1e-12 else math.inf
        ranges.append(r if .05 <= r <= 12. else math.inf)
    rows = []
    for name, scale, bias, offset in (('nominal', 1., 0., 0.),
            ('wheel_minus5pct', .95, 0., 0.), ('wheel_plus5pct', 1.05, 0., 0.),
            ('gyro_minus1deg_s', 1., -math.pi/180, 0.), ('gyro_plus1deg_s', 1., math.pi/180, 0.),
            ('clock_minus5ms', 1., 0., -.005), ('clock_plus5ms', 1., 0., .005)):
        h = MotionHistory()
        for k in range(301):
            t = k/1000.
            v, w = (2., .5) if k < 170 else (5.5, 2.)
            if t + offset >= 0:
                h.add_wheels(t + offset, scale*v/.025, scale*v/.025)
                h.add_gyro(t + offset, w+bias)
        for mode in MODES:
            points, indices, meta = h.project(ranges, -math.pi, 2*math.pi/499, .05, 12., stamp, dt, now, mode, True)
            at_reference = world(points, oracle(meta['reference_stamp_s']))
            as_current = world(points, oracle(now))
            rows.append({'scenario': name, 'mode': mode, 'valid_points': len(indices),
                         'wall_rmse_at_declared_reference_m': float(np.sqrt(np.mean((at_reference[:, 1]-2)**2))),
                         'wall_rmse_if_used_as_current_m': float(np.sqrt(np.mean((as_current[:, 1]-2)**2))),
                         'wheel_scale': scale, 'gyro_error_rad_s': bias, 'timestamp_offset_s': offset})
    nominal = [r for r in rows if r['scenario']=='nominal' and r['mode'] in ('deskew', 'both')]
    passed = all(r['wall_rmse_at_declared_reference_m'] < 1e-10 for r in nominal)
    report = {'passed': passed, 'type': 'synthetic_geometry_not_closed_loop',
              'limitations': ['이상적 수평 정적 벽', '명시한 오차 주입값은 실물 측정치가 아님',
                              '1 kHz 합성 이동 입력이며 실제 100/200 Hz 주행 결과와 분리'], 'rows': rows}
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
