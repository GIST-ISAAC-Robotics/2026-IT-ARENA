import pickle, math, sys
import numpy as np
from arena_autonomy import observation_support as obs
from arena_autonomy.observation_lateral import protected_queries, reachable_from_unknown, polygon_distance
from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.local_path import swept_limit
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
cases = pickle.load(open(sys.argv[1], 'rb'))
def pose(steer, s, cfg):
    k = math.tan(steer)/cfg.wheelbase_m; yaw = k*s
    x = s if k == 0 else math.sin(yaw)/k
    y = 0. if k == 0 else 2*math.sin(yaw/2)**2/k
    wb = cfg.wheelbase_m
    return x - wb/2 + wb/2*math.cos(yaw), y + wb/2*math.sin(yaw), yaw
def body(steer, s, cfg):
    cx, cy, yaw = pose(steer, s, cfg); c, si = math.cos(yaw), math.sin(yaw)
    lx, wy = cfg.length_m/2, cfg.width_m/2
    return Polygon([(cx + c*a - si*b, cy + si*a + c*b) for a, b in ((-lx,-wy),(lx,-wy),(lx,wy),(-lx,wy))])
def swept(steer, dist, cfg, step=.002):
    return unary_union([body(steer, s, cfg) for s in np.r_[np.arange(0, dist, step), dist]])
def role(q, cfg):
    lx, wy = cfg.length_m/2, cfg.width_m/2
    x, y = q
    a = 'front' if x > lx else ('rear' if x < -lx else 'mid')
    b = 'left' if y > wy else ('right' if y < -wy else 'center')
    return a + '_' + b
def wedge(r, length=13.):
    h = math.pi/500; o = np.array(r.ray_origin_xy_m); d = np.array(r.direction_xy)
    rot = lambda t: np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    return Polygon([tuple(o), tuple(o + length*rot(-h)@d), tuple(o + length*rot(h)@d)])
def analyse(c, steer, prev, speed, target, label):
    cfg = c['cfg']; mc = c['motion']; srcs = c['sources']; cur = srcs[0]
    res = obs.sweep_support(srcs, speed, target, steer, prev, cfg, protected_sensor_pose=(mc.lidar_x_m, mc.lidar_y_m, mc.lidar_yaw_rad))
    dist = res.get('required_distance_m')
    print(f'  [{label}] steer {steer:+.4f} prev {prev:+.4f} -> {res["supported"]} {res["reason"]} q {res.get("queries")} sup {res.get("supported_queries")} thin {res.get("thin_intersections")} dist {dist}')
    if dist is None or res['supported']: return res
    q = np.unique(np.vstack([obs.sweep_queries(cur, a, dist, cfg)[0] for a in set((steer, prev))]), axis=0)
    prot = protected_queries(q, (mc.lidar_x_m, mc.lidar_y_m, mc.lidar_yaw_rad))
    sup = np.zeros(len(q), bool); why = np.array(['']*len(q), dtype=object)
    depth_any = np.zeros(len(q), bool)
    for i, s in enumerate(srcs):
        pad = np.array([.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(s.t_ref_s-r.acquired_at_s) for r in s.rays])
        dep = obs.depth_support(s, q, pad); depth_any |= dep
        reach = np.zeros(len(q), bool); reach[prot] = reachable_from_unknown(s, q[prot], cfg)
        sup |= dep & ~reach
    miss = ~sup
    v = max(speed, target); age = cur.t_ref_s - cur.source_first_s
    d_phys = v*(age+.10) + v*v/4
    unk = [r for r in cur.rays if r.status != 'return']
    W = {r.source_index: wedge(r) for r in unk}
    sw = {}
    for name, dd in (('phys', d_phys), ('phys+.06', d_phys+.06), ('full', dist)):
        sw[name] = unary_union([swept(a, dd, cfg) for a in set((steer, prev))])
    cb = body(0, 0, cfg)
    from collections import Counter
    cnt = Counter()
    rows = []
    for j in np.flatnonzero(miss):
        p = Point(q[j]); r = role(q[j], cfg)
        inw = [k for k, w in W.items() if w.buffer(1e-9).contains(p)]
        st = 'masked' if any(cur.rays[k].status == 'masked' for k in inw) else ('unknown' if inw else '')
        ins = tuple(n for n in sw if sw[n].buffer(1e-9).contains(p))
        reason = 'nodepth' if not depth_any[j] else 'lateral'
        cnt[(r, reason, st, ins[0] if ins else 'none', bool(prot[j]))] += 1
        rows.append((q[j], r, reason, st, ins))
    for k, n in sorted(cnt.items(), key=lambda kv: -kv[1]): print('     ', n, k)
    print('     d_phys', round(d_phys,4), 'd_phys+.06', round(d_phys+.06,4), 'full', dist)
    return res, rows
for seq in (3156,):
    c = cases[seq]; d = c['d']
    print('== seq', seq)
    out = analyse(c, 0.18252839482686553, d['previous_steering_rad'], d['speed_mps'], .2, 'recorded')
for seq in (3181, 3501, 3156):
    c = cases[seq]; d = c['d']
    print('== seq', seq, 'speed', d['speed_mps'])
    for st in (-.18, -.12, -.06, -.03, -.01, 0., .01, .03, .06, .12, .18):
        analyse(c, st, st if seq != 3156 else st, d['speed_mps'], .2, f'grid same-prev')
