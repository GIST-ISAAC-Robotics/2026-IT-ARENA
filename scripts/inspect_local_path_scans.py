#!/usr/bin/env python3
"""저장된 정지 스캔의 지역 경로를 재계산한다. Gazebo/구동 명령 없음."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from arena_autonomy.local_path import local_path, pursuit_command

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'artifacts/validation/2026-09-21/local_pursuit'


def main():
    cases = sorted(BASE.glob('lap_*/last_scan.json'))
    fig, axes = plt.subplots(2, (len(cases)+1)//2, figsize=(16, 9), squeeze=False)
    for ax, source in zip(axes.flat, cases):
        scan = json.loads(source.read_text())
        ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
        angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        valid = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
        points = np.column_stack((ranges[valid]*np.cos(angles[valid])+.06, ranges[valid]*np.sin(angles[valid])))
        ax.scatter(points[:, 0], points[:, 1], s=3, color='black')
        for side, color in [('left', 'blue'), ('right', 'orange')]:
            path, meta = local_path(points, side)
            print(source.parent.name, side, {k:v for k,v in meta.items() if k != 'path_points'})
            if path is not None:
                ax.plot(*path.T, color=color, label=side)
        speed, steer, meta = pursuit_command(points, 'left', 0.)
        print('command', speed, steer, {k:v for k,v in meta.items() if k != 'path_points'})
        ax.scatter([0], [0], color='red', marker='>')
        ax.set(xlim=(-.6, 2.), ylim=(-1.5, 1.5), title=source.parent.name, xlabel='forward [m]', ylabel='left [m]')
        ax.set_aspect('equal'); ax.grid(); ax.legend()
    fig.tight_layout()
    fig.savefig(BASE/'stationary_scan_diagnostic.png', dpi=140)


if __name__ == '__main__':
    main()
