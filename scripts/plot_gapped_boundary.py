#!/usr/bin/env python3
"""저장 입력의 실제 벽 조각과 그 지지 경로만 표시한다. 지도·정답 도로 없음."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from arena_autonomy.boundary_evidence import fragment_evidence
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene


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
    cfg = FollowConfig(**events[0]['data']['follow_config'])
    motion = MotionConfig(**events[0]['data']['motion_config'])
    selected = [291, 301, 341, 441]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    notes = []
    for event in events:
        if event['kind'] != 'call' or event['seq'] not in selected:
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        points = np.asarray(scene.occupied_endpoints_xy_m)
        other = 'right' if data['side'] == 'left' else 'left'
        path, _ = local_path(points, other, cfg.wall_offset_m)
        evidence = fragment_evidence(points, path, data['side'], cfg)
        ax = axes.ravel()[selected.index(event['seq'])]
        ax.scatter(points[:, 0], points[:, 1], s=7, color='#777777', label='All occupied returns')
        for i, part in enumerate(evidence['fragments']):
            samples = points[part['source_indices']]
            ax.scatter(samples[:, 0], samples[:, 1], s=13, color='#cf592e' if i == 0 else '#168556',
                       label=f'Observed fragment {i+1}')
        ax.plot(path[:, 0], path[:, 1], color='#2462ad', lw=2, label='Observed opposite-wall path')
        ax.add_patch(plt.Rectangle((-.1, -.075), .2, .15, color='#333b48', alpha=.4))
        ax.set(xlim=(-.25, 1.9), ylim=(-.58, .6), xlabel='Forward x [m]', ylabel='Left y [m]',
               title=f"t={data['reference_s']:.2f} s, observed sum={evidence['observed_ahead_sum_m']:.2f} m")
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
        ax.legend(fontsize=7, loc='lower right')
        notes.append(dict(seq=event['seq'], evidence=evidence))
    if len(notes) != len(selected):
        raise ValueError('selected input events missing')
    fig.suptitle('Gaps remain unknown; fragments are boundary evidence, not driving permission')
    fig.savefig(args.output, dpi=160)
    print(json.dumps(dict(input_sha256=before,
        input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest(), rows=notes), indent=2))


if __name__ == '__main__':
    main()
