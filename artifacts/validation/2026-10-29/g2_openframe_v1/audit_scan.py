"""저장된 실제 GPU 스캔과 평가 자세로 표적 반환을 대조한다. 제어 입력이 아니다."""
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

BASE = Path(__file__).resolve().parent


def first_scan_near(path, stamp):
    with path.open(encoding='utf-8') as stream:
        return min((json.loads(line) for line in stream), key=lambda r: abs(r['stamp_s']-stamp))


def pose_at(rows, stamp):
    return np.array([np.interp(stamp, rows[:, 0], rows[:, i]) for i in (1, 2, 3)])


def inside_distance(point, bounds):
    lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    return np.linalg.norm(np.maximum(np.maximum(lo-point, point-hi), 0.))


def ray_box(origin, direction, lo, hi):
    near, far = 0., float('inf')
    for a in range(2):
        if abs(direction[a]) < 1e-12:
            if not lo[a] <= origin[a] <= hi[a]:
                return None
        else:
            v = sorted(((lo[a]-origin[a])/direction[a], (hi[a]-origin[a])/direction[a]))
            near, far = max(near, v[0]), min(far, v[1])
    return near if far >= near else None


def audit(folder, ax, frame):
    scanpath = folder/'scans_received.jsonl'
    posepath = folder/'ego_poses.jsonl'
    actorpath = folder/'actor_poses.jsonl'
    scan = first_scan_near(scanpath, 5.)  # 출발 전 정지 구간이다.
    poses = np.asarray([json.loads(l) for l in posepath.read_text().splitlines()])
    actors = np.asarray([json.loads(l) for l in actorpath.read_text().splitlines()])
    actor = pose_at(actors, scan['stamp_s'])
    ca, sa = math.cos(actor[2]), math.sin(actor[2])
    rot = np.array([[ca, sa], [-sa, ca]])
    parts = [([x-.01, y-.01], [x+.01, y+.01]) for x in (-.08, .08) for y in (-.055, .055)]
    if not frame:
        parts = [([-.1, -.075], [.1, .075])]
    rows, points = [], []
    for i, r in enumerate(scan['ranges']):
        t = scan['stamp_s']+i*scan['time_increment']
        pose = pose_at(poses, t)
        ce, se = math.cos(pose[2]), math.sin(pose[2])
        origin = rot@(pose[:2]+np.array([.06*ce, .06*se])-actor[:2])
        angle = pose[2]+scan['angle_min']+i*scan['angle_increment']
        direction = rot@np.array([math.cos(angle), math.sin(angle)])
        expected = ray_box(origin, direction, [-.1, -.075], [.1, .075])
        if expected is None:
            continue
        if not isinstance(r, (int, float)) or not math.isfinite(r):
            rows.append(dict(index=i, kind='invalid', expected_box_m=expected))
            continue
        point = origin+float(r)*direction
        gap = min(inside_distance(point, p) for p in parts)
        kind = 'part_return' if gap <= .004 else ('behind_opening' if r > expected+.03 else 'other')
        rows.append(dict(index=i, kind=kind, range_m=float(r), expected_box_m=expected,
                         point_actor_xy=point.tolist(), nearest_part_distance_m=float(gap)))
        points.append(point)
        end = origin+direction*min(r, expected+.28)
        color = {'part_return':'#2c7fb8','behind_opening':'#f28e2b','other':'#777777'}[kind]
        ax.plot([origin[0], end[0]], [origin[1], end[1]], color=color, lw=.9, alpha=.75)
    ax.add_patch(Rectangle((-.1, -.075), .2, .15, fill=False, ec='black', lw=1.8, label='Full body footprint'))
    for lo, hi in parts:
        ax.add_patch(Rectangle(lo, hi[0]-lo[0], hi[1]-lo[1], fc='#777777', alpha=.6))
    if points:
        xy = np.array(points)
        ax.scatter(xy[:, 0], xy[:, 1], s=20, c='#08519c', zorder=5)
    ax.set(xlim=(-.17, .35), ylim=(-.15, .15), aspect='equal',
           xlabel='Target forward x (m)', ylabel='Target left y (m)',
           title='Open frame: actual scan' if frame else 'Solid box: actual scan')
    ax.grid(alpha=.2)
    first, last = pose_at(poses, scan['stamp_s']), pose_at(poses, scan['stamp_s']+.1)
    return dict(scan_stamp_s=scan['stamp_s'], selected_before_green=True,
                ego_translation_during_scan_m=float(np.linalg.norm(last[:2]-first[:2])),
                ego_yaw_change_rad=float(last[2]-first[2]),
                rays_crossing_full_footprint=len(rows),
                counts={kind:sum(r['kind']==kind for r in rows) for kind in ('part_return','behind_opening','invalid','other')},
                source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (scanpath,posepath,actorpath)},
                rays=rows)


if __name__ == '__main__':
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    results = dict(box=audit(BASE.parent/'g2_entry_gazebo_v1/run1', axes[0], False),
                   openframe=audit(BASE/'run1', axes[1], True))
    results['scope'] = 'Evaluation-only transform of raw Gazebo scans; no real LiDAR reflectance proof. 4 mm association tolerance is diagnostic, not control clearance.'
    fig.suptitle('Blue: part returns. Orange: through openings. Gray: unmatched edge sample.')
    fig.tight_layout()
    fig.savefig(BASE/'run1_scan_comparison.png', dpi=150)
    (BASE/'run1_scan_audit.json').write_text(json.dumps(results,indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:{n:r[n] for n in ('scan_stamp_s','ego_translation_during_scan_m','rays_crossing_full_footprint','counts')} for k,r in results.items() if isinstance(r,dict)},indent=2))
