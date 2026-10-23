#!/usr/bin/env python3
"""지정된 저장 판단의 실제 관측 성분과 코너 증거를 회수한다. 새 주행 아님."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_distance import corner_distance_evidence
from profile_follow_trace import SNAPSHOT_MODULES

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def components(p, side):
    sign = 1 if side == 'left' else -1
    order = np.argsort(-sign*np.arctan2(p[:, 1], p[:, 0]), kind='stable')
    a = p[order]
    breaks = np.linalg.norm(np.diff(a, axis=0), axis=1) > (
        .10+.035*np.minimum(np.linalg.norm(a[:-1], axis=1), np.linalg.norm(a[1:], axis=1)))
    rows = []
    for component in np.split(order, np.flatnonzero(breaks)+1):
        mask = (sign*p[component, 1] > .10) & (np.linalg.norm(p[component], axis=1) <= 2.)
        starts = np.flatnonzero(mask & np.r_[True, ~mask[:-1]])
        ends = np.flatnonzero(mask & np.r_[~mask[1:], True])+1
        for lo, hi in zip(starts, ends):
            indices = component[lo:hi]
            if len(indices) < 12:
                continue
            wall = p[indices]
            extrema = wall[np.r_[np.argmin(wall, axis=0), np.argmax(wall, axis=0)]]
            extent = float(np.max(np.linalg.norm(extrema[:, None]-extrema[None], axis=2)))
            rows.append(dict(indices=indices.tolist(), count=len(indices), extent_m=extent,
                             min_xy=wall.min(axis=0).tolist(), max_xy=wall.max(axis=0).tolist()))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--seq', type=int, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.input)
    events = []
    with args.input.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                cfg = FollowConfig(**event['data']['follow_config'])
            elif event['kind'] == 'call' and event['seq'] in args.seq:
                events.append(event)
    if len(events) != len(set(args.seq)):
        raise ValueError('requested call missing')
    fig, axes = plt.subplots((len(events)+1)//2, 2, figsize=(12, 4.5*((len(events)+1)//2)),
                             squeeze=False, constrained_layout=True)
    rows = []
    for event, ax in zip(events, axes.flat):
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        p = np.asarray(scene.occupied_endpoints_xy_m)
        side = data['side']
        other = 'right' if side == 'left' else 'left'
        paths = {}
        ax.scatter(p[:, 0], p[:, 1], s=5, c='#64748b', label='All observed returns')
        for wall, color in ((side, '#dc2626'), (other, '#2563eb')):
            path, info = local_path(p, wall, cfg.wall_offset_m)
            paths[wall] = dict(path=None if path is None else path.tolist(), info=info)
            if path is not None:
                ax.plot(path[:, 0], path[:, 1], color=color, lw=2, label=wall+' path')
        component_rows = components(p, side)
        for index, component in enumerate(component_rows):
            wall = p[component['indices']]
            ax.plot(wall[:, 0], wall[:, 1], lw=2, label=f'inner {index}: {component["extent_m"]:.3f}m')
        reference = paths[other]['path']
        evidence = None if reference is None else corner_distance_evidence(p, reference, side, cfg, detailed=True)
        rows.append(dict(seq=event['seq'], t=data['reference_s'], points=p.tolist(),
                         paths=paths, components=component_rows, evidence=evidence,
                         recorded=event['data']['output']))
        ax.add_patch(plt.Rectangle((-.1, -.075), .2, .15, color='#555', alpha=.3))
        ax.set(xlim=(-1.4, 1.6), ylim=(-1., 1.4), aspect='equal', xlabel='Forward x [m]',
               ylabel='Left y [m]', title=f"{event['seq']} / t={data['reference_s']:.2f}s")
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    for ax in list(axes.flat)[len(events):]:
        ax.set_visible(False)
    fig.savefig(args.output/'geometry.png', dpi=140)
    sources = [f'src/{pkg}/{pkg}/{mod}.py' for pkg, mods in SNAPSHOT_MODULES.items() for mod in mods]
    sources += ['scripts/inspect_corner_geometry.py', 'scripts/profile_follow_trace.py']
    hashes = {p:digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in sources:
            z.write(ROOT/p, p)
    output = dict(scope='saved_sensor_geometry_only', input_sha256=before,
                  input_unchanged=before == digest(args.input), source_sha256=hashes, rows=rows)
    (args.output/'report.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n')
    print(json.dumps([dict(seq=r['seq'], t=r['t'], component_extents=[c['extent_m'] for c in r['components']],
                          reason=r['evidence']['reason'] if r['evidence'] else None) for r in rows]))


if __name__ == '__main__':
    main()
