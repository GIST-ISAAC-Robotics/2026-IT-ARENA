"""정지 장면에서 양측 일치 검사가 왜 거부되는지 함수 단위로 재현하고 호길이 기준 후보를 대조한다.

제어기 소스를 바꾸지 않는다. 후보 함수는 이 파일 안에서만 계산한다.
실행: python3 -I reacquire_probe.py <snapshot_root> <out.json>
"""
import json
import sys
from pathlib import Path

snapshot, out = sys.argv[1], sys.argv[2]
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
    sys.path.insert(0, str(Path(snapshot) / 'src' / pkg))
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import path_agrees, path_agrees_arc
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene

RUNS = {'combined_failed': (ROOT / 'qualifying_profile_v1/run1', 8.0, 53.),
        'rise_only': (ROOT / 'race_acceleration_v1/run1', 8.0, 52.)}


def overlap_stats(a, b):
    pa = [np.asarray(p)[(np.asarray(p)[:, 0] >= .05) & (np.asarray(p)[:, 0] <= .65)] for p in (a, b)]
    if any(len(p) == 0 for p in pa):
        return dict(n=[len(p) for p in pa], ptp=None, overlap=None)
    return dict(n=[len(p) for p in pa], ptp=[float(np.ptp(p[:, 0])) for p in pa],
                overlap=float(min(p[:, 0].max() for p in pa) - max(p[:, 0].min() for p in pa)))


def arc_agreement(a, b, tol, lo=.05, hi=1.0):
    """각 경로를 현재 차체에 가장 가까운 점부터 호길이 lo~hi 구간만 비교한다. 외삽 없음."""
    def clip(p):
        p = np.asarray(p, dtype=float)
        origin = int(np.argmin(np.linalg.norm(p, axis=1)))
        q = p[origin:]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))]
        return q[(arc >= lo) & (arc <= hi)]
    pa, pb = clip(a), clip(b)
    if len(pa) < 5 or len(pb) < 5:
        return False, None
    d = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2)
    err = float(max(np.percentile(d.min(axis=0), 80), np.percentile(d.min(axis=1), 80)))
    return err <= tol, err


result = {}
for name, (folder, lo, hi) in RUNS.items():
    motion = cfg = None
    rows = []
    last_source = None
    with (folder / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] == 'header':
                motion = MotionConfig(**r['data']['motion_config'])
                cfg = r['data']['follow_config']
            if r['kind'] != 'call':
                continue
            t = r['data']['input']['reference_s']
            if not lo <= t <= hi:
                continue
            first = r['data']['input']['scan']['sec'] + r['data']['input']['scan']['nanosec'] * 1e-9
            if first == last_source:
                continue
            last_source = first
            data, scan, history = restore_call(r['data']['input'], motion)
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            except Exception as e:
                continue
            points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
            horizon = max(1.8, .06 + cfg['standstill_gap_m'] + cfg['open_route_speed_mps'] * (.12 + .10) + cfg['open_route_speed_mps'] ** 2 / 4)
            right, ri = local_path(points, 'right', cfg['wall_offset_m'], horizon=horizon)
            left, li = local_path(points, 'left', cfg['wall_offset_m'], horizon=horizon)
            if right is None or left is None:
                rows.append(dict(t=t, none=True))
                continue
            current = path_agrees(right, left, cfg['boundary_agreement_m'])
            _, arc_err = arc_agreement(right, left, cfg['boundary_agreement_m'])
            arc_ok = path_agrees_arc(right, left, cfg['boundary_agreement_m'])
            rows.append(dict(t=round(t, 2), meas=r['data']['input']['speed_mps'], reason=r['data']['output'][2].get('reason'),
                             agree_x_window=bool(current), agree_arc=bool(arc_ok), arc_err=arc_err, overlap=overlap_stats(right, left),
                             right_ahead=ri.get('boundary_observed_ahead_m'), left_ahead=li.get('boundary_observed_ahead_m'),
                             right_wall=ri.get('wall_distance_m'), left_wall=li.get('wall_distance_m')))
    result[name] = rows
with open(out, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scope='현재 스캔의 좌우 경로 일치 재계산. arc 판정은 스크래치 후보 함수이며 arc_err는 같은 호길이 창의 진단값이다. 저장소 제어기에 연결하지 않았다.', runs=result), f, ensure_ascii=False)
rows = [x for x in result['combined_failed'] if x.get('t', 0) >= 46.4]
print('combined stopped scans', len(rows), 'x_window_agree', sum(1 for x in rows if x.get('agree_x_window')), 'arc_agree', sum(1 for x in rows if x.get('agree_arc')))
print(rows[:2])
s = [x for x in result['rise_only'] + result['combined_failed'] if 'agree_arc' in x]
print('all scans', len(s), 'x&arc', sum(1 for x in s if x['agree_x_window'] and x['agree_arc']), 'x_only', sum(1 for x in s if x['agree_x_window'] and not x['agree_arc']),
      'arc_only', sum(1 for x in s if x['agree_arc'] and not x['agree_x_window']), 'neither', sum(1 for x in s if not x['agree_arc'] and not x['agree_x_window']))
