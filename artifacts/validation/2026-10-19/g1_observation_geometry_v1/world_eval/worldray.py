"""평가 전용: 정답 월드 벽으로 무응답 방향의 실제 거리를 확인. 제어 입력 아님."""
import json, math, sys, pickle
import numpy as np
RUN = 'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/'
Z = 0.095
V = []; F = []
for line in open(RUN+'batched_walls_0.obj'):
    p = line.split()
    if not p: continue
    if p[0] == 'v': V.append(tuple(map(float, p[1:4])))
    elif p[0] == 'f': F.append(tuple(int(t.split('/')[0])-1 for t in p[1:4]))
V = np.array(V)
segs = []
for f in F:
    tri = V[list(f)]; pts = []
    for a, b in ((0,1),(1,2),(2,0)):
        za, zb = tri[a,2]-Z, tri[b,2]-Z
        if (za > 0) != (zb > 0):
            t = za/(za-zb); pts.append(tri[a,:2] + t*(tri[b,:2]-tri[a,:2]))
    if len(pts) == 2: segs.append((pts[0], pts[1]))
S = np.array([[a[0],a[1],b[0],b[1]] for a,b in segs])
print('segments', len(S))
def cast(ox, oy, ang, rmax=40.):
    d = np.array([math.cos(ang), math.sin(ang)])
    p = S[:, :2]; e = S[:, 2:]-S[:, :2]
    den = d[0]*e[:,1]-d[1]*e[:,0]
    w = p-np.array([ox, oy])
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[:,0]*e[:,1]-w[:,1]*e[:,0])/den
        u = (w[:,0]*d[1]-w[:,1]*d[0])/den
    ok = (np.abs(den) > 1e-12) & (t > 1e-6) & (u >= 0) & (u <= 1)
    return float(t[ok].min()) if ok.any() else math.inf
ego = [json.loads(l) for l in open(RUN+'ego_poses.jsonl')]
T = np.array([e[0] for e in ego])
def pose_at(t):
    i = int(np.argmin(abs(T-t))); return ego[i]
inc = 2*math.pi/500
cases = pickle.load(open(sys.argv[1], 'rb'))
for seq in (3156, 3181, 3501):
    c = cases[seq]; s = c['scene']
    t, x, y, yaw = pose_at(s.t_ref_s)
    lx, ly = x+.06*math.cos(yaw), y+.06*math.sin(yaw)
    print(f'== seq {seq} t {t} pose ({x:.4f},{y:.4f},{math.degrees(yaw):.2f}deg)')
    rows = []
    for r in s.rays:
        if r.status != 'unknown': continue
        bearing = math.atan2(r.direction_xy[1], r.direction_xy[0])
        dist = cast(lx, ly, yaw+bearing)
        rows.append((r.source_index, round(math.degrees(bearing),2), round(dist,3)))
    print('  unknown rays true distance (current-pose approx):', rows)
    # all rays > 12 m within ±100 at current pose (static approx)
    far = []
    for k in range(500):
        b = -math.pi + k*inc
        if abs(b) > math.radians(100): continue
        dd = cast(lx, ly, yaw+b)
        if dd > 12.: far.append((k, round(math.degrees(b),2), round(dd,2)))
    print('  static rays >12m within ±100:', far)
# continuation along official centerline
cl = np.genfromtxt('src/arena_gazebo/worlds/it_arena_official/centerline.csv', delimiter=',', names=True)
xy = np.column_stack((cl['x_m'], cl['y_m'])); sv = cl['s_m']
t, x, y, yaw = pose_at(cases[3501]['scene'].t_ref_s)
i0 = int(np.argmin(np.sum((xy-[x,y])**2, axis=1)))
print('nearest centerline s', sv[i0], 'err', np.hypot(*(xy[i0]-[x,y])))
for k in range(i0-20, min(i0+120, len(xy)-1), 4):
    hx, hy = xy[k+1]-xy[k]; h = math.atan2(hy, hx)
    lx, ly = xy[k][0]+.06*math.cos(h), xy[k][1]+.06*math.sin(h)
    far = []
    for j in range(500):
        b = -math.pi + j*inc
        if abs(b) > math.radians(100): continue
        if cast(lx, ly, h+b) > 12.: far.append(round(math.degrees(b),1))
    print(f'  s {sv[k]:.2f} heading {math.degrees(h):7.2f} far>12m bearings: {far}')
