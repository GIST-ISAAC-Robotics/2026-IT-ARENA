"""단절 지점 기준(한쪽이 벽에 붙은 지주 포함) 시선 깊이 판정 후보 비교. 저장소 쓰기 없음."""
import sys, json, math, pickle, re
from pathlib import Path
sys.dont_write_bytecode = True
ROOT = Path('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local')
SNAP = ROOT/'build/r81_frozen_diagnosis'
OUT = Path(__file__).resolve().parent
RUN = ROOT/'artifacts/validation/2026-11-11/native_nonreturn_r81_v1/two_actor_run1/run1'
sys.path[:0] = [str(SNAP/'src/arena_autonomy'), str(SNAP/'src/arena_vehicle_interface'), str(SNAP/'scripts')]
import numpy as np
import arena_autonomy.local_path as lp
from arena_autonomy.local_follow import FollowConfig, anchor_error
from arena_autonomy.corner_distance import corner_distance_evidence

data = pickle.load(open(OUT/'clouds.pkl', 'rb'))
first, decisions = data['first'], data['decisions']
cfg = FollowConfig(**data['follow_config'])
base_no = lp.narrow_occluders

def limit(a, b):
    return .10+.035*min(np.linalg.norm(a), np.linalg.norm(b))

def anchored(min_depth=.03, max_points=6, cross=.04, radial=.10, isolated_only=False):
    """단절 하나에 닿은 1~max_points점 묶음(앞쪽 또는 뒤쪽)을 시선 깊이로 판정한다."""
    def fn(points, broken_unused):
        n = len(points)
        alive = np.ones(n, dtype=bool)
        joins = []
        for _ in range(12):
            idx = np.flatnonzero(alive)
            p = points[idx]
            if len(p) < 3:
                break
            gaps = np.linalg.norm(np.diff(p, axis=0), axis=1)
            lim = .10+.035*np.minimum(np.linalg.norm(p[:-1], axis=1), np.linalg.norm(p[1:], axis=1))
            broken = gaps > lim
            for b, _ in joins:  # 이미 이은 곳은 다시 끊긴 것으로 보지 않는다
                pos = np.searchsorted(idx, b)
                if pos < len(broken) and idx[pos] == b:
                    broken[pos] = False
            found = None
            for i in np.flatnonzero(broken):
                for k in range(1, max_points+1):
                    for lo, hi in ((i+1, i+1+k), (i-k+1, i+1)):
                        if lo < 1 or hi >= len(p):
                            continue
                        if isolated_only and not (broken[lo-1] and broken[hi-1]):
                            continue
                        cluster, before, after = p[lo:hi], p[lo-1], p[hi]
                        if np.linalg.norm(after-before) > 2*limit(before, after):
                            continue
                        if min(np.linalg.norm(before), np.linalg.norm(after))-np.max(np.linalg.norm(cluster, axis=1)) < min_depth:
                            continue
                        center = cluster.mean(axis=0)
                        los = center/max(1e-9, np.linalg.norm(center))
                        if np.ptp(cluster @ np.array([-los[1], los[0]])) > cross or np.ptp(cluster @ los) > radial:
                            continue
                        found = (lo, hi)
                        break
                    if found:
                        break
                if found:
                    break
            if not found:
                break
            lo, hi = found
            alive[idx[lo:hi]] = False
            joins.append((int(idx[lo-1]), int(idx[hi])))
        return ~alive, joins
    return fn

VARIANTS = {
    'baseline': base_no,
    'isolated_los_d03': anchored(.03, isolated_only=True),
    'anchored_los_d03': anchored(.03),
    'anchored_los_d06': anchored(.06),
    'anchored_los_d06_cross06': anchored(.06, cross=.06),
}

def evaluate(cloud, kw, args, side='right'):
    path, info = lp.local_path(cloud, side, *args, **kw)
    row = dict(reason=info['reason'], model=info.get('wall_model'), ahead=info.get('boundary_observed_ahead_m'))
    row['sufficient'] = bool(path is not None and info['boundary_observed_ahead_m'] >= cfg.boundary_min_ahead_m
                             and anchor_error(info, cfg) <= cfg.boundary_anchor_error_m)
    row['corner_distance'] = None
    if row['sufficient']:
        row['corner_distance'] = corner_distance_evidence(cloud, path, 'left' if side == 'right' else 'right', cfg)['reason']
    row['eligible'] = row['corner_distance'] == 'observed_corner_distance_support'
    return row, path

report = {}
ts = sorted(t for (t, s) in first if s == 'right' and 43.6 <= t <= 48.3)
win = {}
for name, fn in VARIANTS.items():
    lp.narrow_occluders = fn
    seq = [evaluate(first[(t, 'right')]['cloud'], first[(t, 'right')]['kw'], first[(t, 'right')]['args'])[0] for t in ts]
    lp.narrow_occluders = base_no
    flags = [r['eligible'] for r in seq]
    longest = cur = 0
    for f in flags:
        cur = 0 if f else cur+1; longest = max(longest, cur)
    kinds = {}
    for r in seq:
        key = ('eligible' if r['eligible'] else 'no_path:'+r['reason'] if r['reason'] != 'local_path' else
               'chain_short' if not r['sufficient'] else 'connected_but_generic_curve' if r['model'] is None else
               'corner_ok_distance:'+str(r['corner_distance']))
        kinds[key] = kinds.get(key, 0)+1
    win[name] = dict(eligible=sum(flags), total=len(flags),
                     consecutive_pairs=sum(1 for a, b in zip(flags[:-1], flags[1:]) if a and b),
                     longest_fail_run=longest, kinds=kinds,
                     fail_times=[round(t, 2) for t, f in zip(ts, flags) if not f])
report['window_variants'] = win

# 전체 주행 감사
def load_poses(path):
    return np.array([json.loads(line) for line in open(path, encoding='utf-8')])
ego = load_poses(RUN/'ego_poses.jsonl')
actors = {'eval_g1b_actor': load_poses(RUN/'actor_poses.jsonl')}
for k, v in json.load(open(RUN/'additional_actors.json', encoding='utf-8'))['poses'].items():
    actors[k] = np.array(v)
def pose_at(rows, t):
    return rows[min(len(rows)-1, int(np.searchsorted(rows[:, 0], t)))]
def actor_distance(center, t):
    e = pose_at(ego, t)
    c, s = math.cos(e[3]), math.sin(e[3])
    w = np.array([e[1]+c*center[0]-s*center[1], e[2]+s*center[0]+c*center[1]])
    return min(float(np.linalg.norm(w-pose_at(a, t)[1:3])) for a in actors.values())
def chain(points, side):
    sign = 1 if side == 'left' else -1
    p = points[np.argsort(-sign*np.arctan2(points[:, 1], points[:, 0]))]
    gaps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(p[:-1], axis=1), np.linalg.norm(p[1:], axis=1))
    return p, gaps > limits
cls = lambda path, info: ('none:'+info['reason'] if path is None else
                          ('corner' if info.get('wall_model') else 'curve')+('_long' if info['boundary_observed_ahead_m'] >= .65 else '_short'))
audit = {}
for name in ('anchored_los_d03', 'anchored_los_d06'):
    fn = VARIANTS[name]
    stats = dict(new_clusters=0, new_points=0, dropped_baseline_points=0, min_actor_distance=math.inf,
                 clusters_within_0p35_of_actor=0, max_cluster_range=0., max_bridge_chord=0.)
    sizes = {}; transitions = {}; changed = []; deviation = []
    for (t, side), c in sorted(first.items()):
        if c['kw'].get('narrow_occluder_bridge') is not True:
            continue
        pts = c['cloud'][np.all(np.isfinite(c['cloud']), axis=1)]
        if len(pts) < 12:
            continue
        p, broken = chain(pts, side)
        m0, _ = base_no(p, broken); m1, j1 = fn(p, broken)
        stats['dropped_baseline_points'] += int((m0 & ~m1).sum())
        for a, b in j1:
            stats['max_bridge_chord'] = max(stats['max_bridge_chord'], float(np.linalg.norm(p[b]-p[a])))
        new = m1 & ~m0
        if not new.any() and not (m0 & ~m1).any():
            continue
        idx = np.flatnonzero(new)
        if len(idx):
            for g in np.split(idx, np.flatnonzero(np.diff(idx) > 1)+1):
                center = p[g].mean(axis=0)
                da = actor_distance(center, t)
                stats['new_clusters'] += 1; stats['new_points'] += len(g)
                sizes[len(g)] = sizes.get(len(g), 0)+1
                stats['min_actor_distance'] = min(stats['min_actor_distance'], da)
                stats['clusters_within_0p35_of_actor'] += da < .35
                stats['max_cluster_range'] = max(stats['max_cluster_range'], float(np.linalg.norm(center)))
        lp.narrow_occluders = base_no
        path0, info0 = lp.local_path(c['cloud'], side, *c['args'], **c['kw'])
        lp.narrow_occluders = fn
        path1, info1 = lp.local_path(c['cloud'], side, *c['args'], **c['kw'])
        lp.narrow_occluders = base_no
        k0, k1 = cls(path0, info0), cls(path1, info1)
        transitions[k0+' -> '+k1] = transitions.get(k0+' -> '+k1, 0)+1
        dev = None
        if path0 is not None and path1 is not None:
            a = path0[(path0[:, 0] > 0) & (np.linalg.norm(path0, axis=1) < 1.0)]
            if len(a):
                dev = float(np.max(np.min(np.linalg.norm(a[:, None]-path1[None], axis=2), axis=1)))
                deviation.append(dev)
        worse = (k0.endswith('_long') and not k1.endswith('_long')) or (k0 == 'corner_long' and k1 != 'corner_long') or (dev or 0) > .10
        if worse:
            d = decisions.get(t, {})
            changed.append(dict(t=round(t, 2), side=side, follow_side=d.get('side'), kind=k0+' -> '+k1,
                                ahead=[info0.get('boundary_observed_ahead_m'), info1.get('boundary_observed_ahead_m')],
                                reason1=info1.get('reason'), dev=dev, decision=d.get('baseline_reason'),
                                speed=d.get('speed'),
                                new=[p[g].mean(axis=0).round(3).tolist() for g in np.split(idx, np.flatnonzero(np.diff(idx) > 1)+1)] if len(idx) else []))
    audit[name] = dict(stats=stats, cluster_sizes=sizes, path_transitions=transitions, worse_or_large_change=changed,
                       deviation=dict(n=len(deviation), max=max(deviation) if deviation else None,
                                      p95=float(np.percentile(deviation, 95)) if deviation else None))
report['run_audit'] = audit

gain = {}
lp.narrow_occluders = VARIANTS['anchored_los_d06']
for t, d in sorted(decisions.items()):
    if d['baseline_reason'] != 'follow_boundary_unconfirmed' or d['cand'] is None:
        continue
    res = []
    for side in ('left', 'right'):
        c = first.get((t, side))
        if c is None:
            res.append('not_called'); continue
        b = c['info']
        b_long = b['reason'] == 'local_path' and b['boundary_observed_ahead_m'] >= .65
        path, info = lp.local_path(c['cloud'], side, *c['args'], **c['kw'])
        res.append(('long' if b_long else 'short')+'>'+('long' if path is not None and info['boundary_observed_ahead_m'] >= .65 else 'short'))
    gain[' | '.join(res)] = gain.get(' | '.join(res), 0)+1
lp.narrow_occluders = base_no
report['unconfirmed_fresh_scans_left_right_baseline_to_anchored_d06'] = gain
(OUT/'analyze2_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float)+'\n', encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, default=float))
