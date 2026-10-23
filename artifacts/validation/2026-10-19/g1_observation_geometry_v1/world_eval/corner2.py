import json, math, sys
import numpy as np
exec(open('artifacts/validation/2026-10-19/g1_observation_geometry_v1/world_eval/worldray.py').read().split("ego = [")[0])
cl = np.genfromtxt('src/arena_gazebo/worlds/it_arena_official/centerline.csv', delimiter=',', names=True)
for k in range(len(cl)):
    if 40.4 <= cl['s_m'][k] <= 42.6 and k % 2 == 0:
        print(f"s {cl['s_m'][k]:.2f} x {cl['x_m'][k]:.3f} y {cl['y_m'][k]:.3f}")
# horizontal cross-section: wall hits along x at fixed y, along y at fixed x
def hits_line(p, d):
    out = []
    for s in S:
        a = np.array(s[:2]); e = np.array(s[2:])-a
        den = d[0]*e[1]-d[1]*e[0]
        if abs(den) < 1e-12: continue
        w = a-np.array(p)
        t = (w[0]*e[1]-w[1]*e[0])/den; u = (w[0]*d[1]-w[1]*d[0])/den
        if 0 <= u <= 1: out.append(round(float(t), 3))
    return sorted(set(out))
for y in (0.9, 1.1, 1.3, 1.5, 1.7, 2.0):
    h = [round(8.5+t, 3) for t in hits_line((8.5, y), (1, 0)) if 0 <= t <= 2.2]
    print(f'walls crossing y={y}: x =', h)
for x in (9.3, 9.5, 9.6, 9.7, 9.8, 9.9, 10.0):
    h = [round(0.4+t, 3) for t in hits_line((x, 0.4), (0, 1)) if 0 <= t <= 2.5]
    print(f'walls crossing x={x}: y =', h)
# fine band region
dirs = np.radians(np.arange(60, 120, .25))
print('band region (lidar positions with >12m dir in 60..120 deg world):')
for y in np.arange(0.9, 1.8, 0.05):
    row = ''
    for x in np.arange(9.3, 10.05, 0.025):
        f = any(cast(x, y, a) > 12. for a in dirs)
        row += '#' if f else '.'
    print(f'y {y:.2f} ' + row)
print('x axis from 9.300 step 0.025')
