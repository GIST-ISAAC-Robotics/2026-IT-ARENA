"""탐색용: 재개 등속 실행 scan 8.502(8.62초)의 왼쪽 경계 실패 이유(제어 변경 없음)."""
import json, math, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import replay_g1b_follow as rp  # noqa: E402
m = rp.load_modules(ROOT)
from arena_autonomy.local_follow import boundary_support, drop_protrusions, FollowConfig  # noqa: E402
from arena_autonomy.scene_motion import transport_points  # noqa: E402

run = ROOT/'artifacts/validation/2026-10-07/g1b_resume_v1_slow_s0'
target = float(sys.argv[1]) if len(sys.argv) > 1 else 8.62
captured = {}
original = m.LocalFollow.command


def spy(self, scene, history, *args, **kwargs):
    if abs(scene.t_ref_s-target) < 1e-6:
        captured['points'] = np.asarray(scene.occupied_endpoints_xy_m)
        if self.path is not None:
            captured['carried'] = transport_points(self.path, history, self.path_ref, scene.t_ref_s)
    return original(self, scene, history, *args, **kwargs)


m.LocalFollow.command = spy
rp.replay(run, m, target-1.5, target)
pts, cfg = captured['points'], FollowConfig()
for name, sel in (('unfiltered', pts),):
    path, info = m.local_path(sel, 'left', .425)
    print(name, info.get('reason'), info.get('boundary_observed_ahead_m'))
if 'carried' in captured:
    s1 = boundary_support(pts, captured['carried'], 'left', cfg)
    print('pass1', m.local_path(s1, 'left', .425)[1].get('reason'), len(s1))
    s2 = drop_protrusions(s1, captured['carried'], 'left', cfg)
    print('pass2', m.local_path(s2, 'left', .425)[1].get('reason'), len(s2))
q = pts[(pts[:, 1] > .1) & (pts[:, 0] > -.75) & (pts[:, 0] < .9)]
q = q[np.argsort(-np.arctan2(q[:, 1], q[:, 0]))]
prev = None
for p in q:
    gap = None if prev is None else float(np.linalg.norm(p-prev))
    lim = None if prev is None else .10+.035*min(np.linalg.norm(p), np.linalg.norm(prev))
    flag = 'BREAK' if gap is not None and gap > lim else ''
    near = abs(p[0]) < .35
    print(f'{p[0]:7.3f} {p[1]:7.3f} {"cand" if near else "    "} gap {gap if gap is None else round(gap, 3)} {flag}')
    prev = p


def explain(points, side='left', offset=.425, horizon=1.8):
    """local_path 내부 단계를 그대로 따라가며 선택 구간·창·잔차를 출력(공용 함수 불변)."""
    sign = 1 if side == 'left' else -1
    points = points[np.all(np.isfinite(points), axis=1)]
    angles = np.arctan2(points[:, 1], points[:, 0])
    points = points[np.argsort(-sign*angles)]
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
    breaks = np.flatnonzero(gaps > limits)
    for wall in np.split(points, breaks+1):
        if len(wall) < 12:
            continue
        candidates = np.flatnonzero((np.abs(wall[:, 0]) < .35) & (sign*wall[:, 1] > .10))
        if not len(candidates):
            continue
        d = np.linalg.norm(wall[candidates]-[0., sign*offset], axis=1)
        anchor = int(candidates[np.argmin(d)])
        chord = wall[2:]-wall[:-2]; middle = wall[1:-1]-wall[:-2]
        rough = float(np.median(np.abs(chord[:, 0]*middle[:, 1]-chord[:, 1]*middle[:, 0])/np.maximum(1e-9, np.linalg.norm(chord, axis=1))))
        print('segment', len(wall), 'x', round(wall[0, 0], 3), '..', round(wall[-1, 0], 3), 'anchor', np.round(wall[anchor], 3),
              'score', round(float(d.min()), 3), 'roughness', round(rough, 4))
        metric = wall
        if rough > .003:
            padded = np.pad(wall, ((3, 3), (0, 0)), mode='edge')
            metric = np.column_stack([np.convolve(padded[:, a], np.ones(7)/7, mode='valid') for a in (0, 1)])
        arc = np.r_[0., np.cumsum(np.maximum(1e-6, np.linalg.norm(np.diff(metric, axis=0), axis=1)))]
        first, last = max(0., arc[anchor]-.25), min(arc[-1], arc[anchor]+horizon)
        inwin = wall[(arc >= first) & (arc <= last)]
        print('   window arc', round(first, 3), round(last, 3), 'x range', round(inwin[:, 0].min(), 3), round(inwin[:, 0].max(), 3),
              'inward pts(<.38)', int(np.sum(sign*inwin[:, 1] < .38)), 'pts', len(inwin))


print('--- unfiltered')
explain(pts)
if 'carried' in captured:
    print('--- pass1')
    explain(s1)

if 'carried' in captured:
    c = captured['carried']
    print('--- carried x', np.round(c[[0, 1, 2, -1], 0], 3), 'y', np.round(c[[0, 1, 2, -1], 1], 3))
    tangent = np.gradient(c, axis=0); tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    wall = c+.425*np.column_stack((-tangent[:, 1], tangent[:, 0]))
    print('expected wall first pts', np.round(wall[:3], 3))
    post = pts[(pts[:, 0] > .05) & (pts[:, 0] < .15) & (pts[:, 1] > .3) & (pts[:, 1] < .36)]
    for p in post:
        dist = np.linalg.norm(wall-p, axis=1); k = int(np.argmin(dist))
        longitudinal = float((p-wall[k])@tangent[k])
        print('post pt', np.round(p, 3), 'nearest idx', k, 'd', round(float(dist[k]), 3), 'long', round(longitudinal, 3))
