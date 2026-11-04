"""우회 최초 진입의 현재 반환점과 전이 경로 간섭. 평가 전용이며 제어기 변경 없음."""
from pathlib import Path
import json
import math
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.bypass_entry_trial import preview_clearance, preview_point_gap

CASES = [('working_p1', 'official_traffic_v1/p1'), ('working_r1', 'official_traffic_v1/r1'),
         ('blocked_pole', 'official_race_distance_v1/run1')]


def first_entry(path):
    for line in path.open(encoding='utf-8'):
        event = json.loads(line)
        if event['kind'] == 'header':
            header = event['data']
        elif event['kind'] == 'call':
            detail = event['data']['output'][2]
            if detail.get('mode_before') == 'FOLLOW' and detail.get('handoff_state') == 'BYPASS':
                return header, event['data']
    raise ValueError('entry not recorded')


def inspect(name, relative, ax):
    header, call = first_entry(HERE.parent/relative/'follow_decisions.jsonl')
    motion = MotionConfig(**header['motion_config'])
    cfg = FollowConfig(**header['follow_config'])
    data, scan, history = restore_call(call['input'], motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    detail = call['output'][2]['bypass']
    chosen = detail['selected']
    row = next(r for r in detail['candidates'] if r['offset_m'] == chosen['offset_m']
               and r['transition_m'] == chosen['transition_m'])
    path = np.asarray(row['path_points'])
    # 전이 완성 위치를 실제 경로점 사이에서 보간. 이후의 계속 주행 구간은 이번 비교에서 제외.
    end = path[0, 0]+chosen['transition_m']
    prefix = np.vstack([path[path[:, 0] < end], [end, np.interp(end, path[:, 0], path[:, 1])]])
    points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
    fixed = .025+cfg.measurement_padding_m
    padding = np.full(len(points), fixed)
    result = dict(name=name, reference_s=data['reference_s'], selected=chosen,
        measured_speed_mps=data['speed_mps'], fixed_padding_m=fixed,
        actual_original_preview_gap_m=row['preview_point_gap_m'],
        actual_original_preview_progress_m=row['preview_return_progress_m'],
        fixed_transition_gap_m=preview_point_gap(prefix, points, padding, cfg),
        fixed_transition_progress_m=preview_clearance(prefix, points, padding, cfg),
        body_only_transition_gap_m=preview_point_gap(prefix, points, np.zeros(len(points)), cfg),
        fixed_full_path_gap_m=preview_point_gap(path, points, padding, cfg),
        scope='현재 반환과 미래 표본 차체의 간섭 비교. 정지물체/미래 자유/실행 허가 증명 아님.')
    delta = np.gradient(prefix, axis=0)
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    centers = prefix+cfg.wheelbase_m/2*np.c_[np.cos(yaw), np.sin(yaw)]
    diff = points[None, :, :]-centers[:, None, :]
    x = np.cos(yaw[:, None])*diff[:, :, 0]+np.sin(yaw[:, None])*diff[:, :, 1]
    y = -np.sin(yaw[:, None])*diff[:, :, 0]+np.cos(yaw[:, None])*diff[:, :, 1]
    hits = np.any((np.abs(x) <= cfg.length_m/2+fixed) & (np.abs(y) <= cfg.width_m/2+fixed), axis=0)
    ax.scatter(points[:, 0], points[:, 1], s=5, c='#637c8d', label='measured returns')
    ax.scatter(points[hits, 0], points[hits, 1], s=12, c='#df3939', label='transition interference')
    ax.plot(prefix[:, 0], prefix[:, 1], c='#148067', lw=2, label='rear axle transition')
    ax.plot(centers[:, 0], centers[:, 1], '--', c='#d08c20', label='body center')
    ax.set(xlim=(-.15, 1.6), ylim=(-.65, .65), aspect='equal',
           title=f'{name}: fixed gap {result["fixed_transition_gap_m"]:.3f} m',
           xlabel='vehicle forward [m]', ylabel='vehicle left [m]')
    ax.grid(alpha=.2)
    return result


if __name__ == '__main__':
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    rows = [inspect(name, path, ax) for (name, path), ax in zip(CASES, axes)]
    axes[0].legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE/'entry_comparison.png', dpi=150)
    with (HERE/'comparison.json').open('x', encoding='utf-8') as stream:
        json.dump(rows, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(rows, ensure_ascii=False))
