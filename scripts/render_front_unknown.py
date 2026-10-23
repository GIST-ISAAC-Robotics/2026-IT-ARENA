#!/usr/bin/env python3
"""저장 광선 진단을 도식화한다. Gazebo 스크린샷/센서 영상이 아니다."""
import argparse
import json
import math
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from diagnose_front_unknown import ROOT, digest, wall_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagnostic', type=Path)
    parser.add_argument('world', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.diagnostic.read_text())
    inputs = {str(args.diagnostic): digest(args.diagnostic), str(args.world): digest(args.world)}
    if source['input_sha256'].get(str(args.world)) != inputs[str(args.world)] or not source['input_unchanged']:
        raise ValueError('diagnostic/world identity mismatch')
    boxes, _, meshes = wall_geometry(args.world)
    inputs.update({str(p): digest(p) for p in meshes})
    example = source['examples'][1]
    selected = {}
    for row in example['casts']:
        selected.setdefault(row['index'], row)
    fig, axes = plt.subplots(1, 2, figsize=(12, 7), constrained_layout=True)
    origin = np.asarray(next(iter(selected.values()))['origin_xyz_m'][:2])
    for ax in axes:
        for box in boxes:
            polygon = np.asarray(box.footprint.polygon(box.x_m, box.y_m, box.yaw_rad))
            ax.fill(polygon[:, 0], polygon[:, 1], color='#64748b', alpha=.55)
        first = True
        for row in selected.values():
            direction = np.array([math.cos(row['world_angle_rad']), math.sin(row['world_angle_rad'])])
            p = np.asarray(row['origin_xyz_m'][:2])
            missing = row['raw_kind'] == 'positive_infinity'
            length = row['collision_distance_m']
            if length is None:
                continue
            q = p+direction*length
            if missing:
                ax.plot([p[0], q[0]], [p[1], q[1]], '--', color='#ea580c', lw=1.8,
                        label='Unknown (+Inf), geometry only' if first else None)
                clip = p+direction*12.
                ax.scatter(*clip, marker='x', color='#dc2626', s=35,
                           label='12 m measurement limit' if first else None)
                ax.scatter(*q, marker='o', facecolors='none', edgecolors='#ea580c', s=35,
                           label='Wall hit beyond range' if first else None)
                first = False
            else:
                q = p+direction*min(length, 12.)
                ax.plot([p[0], q[0]], [p[1], q[1]], color='#94a3b8', lw=1.)
        ax.scatter(*origin, s=45, c='#0f172a', zorder=5, label='LiDAR origin')
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
        ax.set_xlabel('World x [m]')
        ax.set_ylabel('World y [m]')
    axes[0].set_title('Whole-course geometry and missing side rays')
    axes[0].legend(fontsize=8, loc='upper left')
    axes[1].set(xlim=(origin[0]-.75, origin[0]+.75), ylim=(origin[1]-.65, origin[1]+.85),
                title='Local view: about +89 deg from the sensor')
    # 범례와 별도로 현재 차량의 전방을 센서 광선과 구별해 표시한다.
    ray = selected[min(selected)]
    raw_angle = -math.pi+ray['index']*math.tau/500
    yaw = ray['world_angle_rad']-raw_angle
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    axes[1].annotate('Vehicle forward', xy=origin+.5*forward, xytext=origin+.1*forward,
                     arrowprops=dict(arrowstyle='->', color='#2563eb', lw=2), color='#2563eb', fontsize=9)
    fig.suptitle(f"Recorded first stop input: t={example['time_s']:.2f} s\n"
                 'Static planar ray diagnostic; not a rendered sensor image or free-space proof', fontsize=12)
    fig.savefig(args.output/'unknown_range_geometry.png', dpi=150)
    plt.close(fig)
    paths = [Path(__file__).resolve(), ROOT/'scripts/diagnose_front_unknown.py', ROOT/'scripts/g0_geometry.py']
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in paths}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(p, p.relative_to(ROOT))
    metadata = dict(scope='derived_diagnostic_figure_not_gazebo_screenshot', input_sha256=inputs,
        input_unchanged=all(digest(p) == h for p, h in inputs.items()), source_sha256=hashes,
        source_unchanged=all(digest(ROOT/p) == h for p, h in hashes.items()),
        missing_rays_filled=False, runtime_changed=False)
    (args.output/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata))


if __name__ == '__main__':
    main()
