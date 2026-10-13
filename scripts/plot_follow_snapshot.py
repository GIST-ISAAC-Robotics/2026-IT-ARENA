#!/usr/bin/env python3
"""기록된 G1 광선/운동 이력으로만 경계 후보를 그린다. 지도·정답 외곽 없음."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from arena_autonomy.follow_trace import replay_call, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow, boundary_support, drop_protrusions
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene, transport_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    with args.input.open() as stream:
        events = [json.loads(line) for line in stream]
    cfg = FollowConfig(**events[0]['data']['follow_config'])
    motion = MotionConfig(**events[0]['data']['motion_config'])
    follow = LocalFollow(cfg)
    # 최초 경계 갱신 실패/만료와 재획득 전후. 이 기록의 seq 선택이며 새 제어 조건이 아니다.
    selected = [291, 296, 331, 436]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    notes = []
    for event in events[1:]:
        if event['kind'] == 'reset':
            follow.reset()
        elif event['kind'] == 'call':
            if event['seq'] in selected:
                ax = axes.ravel()[selected.index(event['seq'])]
                data, scan, history = restore_call(event['data']['input'], motion)
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=True,
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                points = np.asarray(scene.occupied_endpoints_xy_m)
                ax.scatter(points[:, 0], points[:, 1], s=5, color='#606a74', label='All occupied returns')
                candidates = {}
                for side, color in [('left', '#df453d'), ('right', '#167ed0')]:
                    path, info = local_path(points, side, cfg.wall_offset_m)
                    candidates[side] = {k: v for k, v in info.items() if k != 'path_points'}
                    if path is not None:
                        ax.plot(path[:, 0], path[:, 1], color=color, lw=2, label=side+' candidate')
                        if side == 'right':
                            selected_points = boundary_support(points, path, 'left', cfg)
                            refined = drop_protrusions(selected_points, path, 'left', cfg)
                            refined_path, refined_info = local_path(refined, 'left', cfg.wall_offset_m)
                            if refined_path is not None:
                                ax.plot(refined_path[:, 0], refined_path[:, 1], color='#9c4bbe', label='Left reselected')
                            candidates['reselected_left'] = {k: v for k, v in refined_info.items() if k != 'path_points'}
                if follow.path is not None and data['reference_s']-follow.path_source < cfg.memory_ttl_s:
                    carried = transport_points(follow.path, history, follow.path_ref, data['reference_s'])
                    ax.plot(carried[:, 0], carried[:, 1], '--', color='#df9300', lw=2, label='Valid carried path')
                ax.add_patch(plt.Rectangle((-.1, -.075), .2, .15, color='#333b48', alpha=.35))
                ax.set(xlim=(-.2, 1.8), ylim=(-.60, .60), xlabel='Forward x [m]', ylabel='Left y [m]',
                       title=f"seq {event['seq']}, t={data['reference_s']:.2f} s")
                ax.set_aspect('equal')
                ax.grid(alpha=.2)
                ax.legend(fontsize=7, loc='lower right')
                notes.append({'seq': event['seq'], 'reference_s': data['reference_s'], 'candidates': candidates})
            replay_call(follow, event['data']['input'], motion)
    fig.suptitle('Recorded sensor-only boundary candidates (not ground-truth road)')
    fig.savefig(args.output, dpi=160)
    print(json.dumps(notes, indent=2))


if __name__ == '__main__':
    main()
