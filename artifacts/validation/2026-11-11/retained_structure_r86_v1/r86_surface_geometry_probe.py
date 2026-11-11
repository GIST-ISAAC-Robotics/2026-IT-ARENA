"""Read-only finite local surface diagnostic; never grants controller permission."""
import json
from pathlib import Path
import numpy as np

root = Path(__file__).resolve().parents[1]
folder = root/'artifacts/validation/2026-11-11/guard_masked_speed_r84_v1'
fixture = json.loads((folder/'latch_sensor_fixture.json').read_bytes())
points = np.array(fixture['context']['current_return_points_xy'])
ids = np.array(fixture['context']['current_return_source_indices'])
rows = []
seen = set()
for witness in fixture['witnesses']:
    key = tuple(witness['previous_point_now_xy'])
    if key in seen:
        continue
    seen.add(key)
    point = np.array(key)
    distances = np.linalg.norm(points-point, axis=1)
    nearest = np.argsort(distances)[:5]
    patches = []
    for index in nearest:
        for width in (9, 15, 21, 31):
            start = max(0, index-width//2)
            patch = points[start:start+width]
            source = ids[start:start+width]
            if len(patch) != width or np.any(np.diff(source) != 1):
                continue
            center = patch.mean(axis=0)
            _, _, axes = np.linalg.svd(patch-center, full_matrices=False)
            tangent, normal = axes
            along = (patch-center)@tangent
            proposed = float((point-center)@tangent)
            patches.append(dict(source_first=int(source[0]), count=width,
                span_m=float(np.ptp(along)), finite=bool(min(along)<=proposed<=max(along)),
                normal_m=float(abs((point-center)@normal)),
                rms_m=float(np.sqrt(np.mean(((patch-center)@normal)**2))),
                max_residual_m=float(max(abs((patch-center)@normal))),
                max_chord_m=float(max(np.linalg.norm(np.diff(patch,axis=0),axis=1)))))
    patches.sort(key=lambda p: (not p['finite'], p['normal_m']))
    rows.append(dict(previous_xy=key, nearest_distance_m=float(min(distances)), patches=patches[:5]))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(8,6))
old=np.array(fixture['context']['previous_points_now_xy'])
ax.scatter(old[:,0],old[:,1],s=12,alpha=.5,label='Previous projected returns')
ax.scatter(points[:,0],points[:,1],s=12,alpha=.7,label='Current returns')
t=np.array(list(seen));ax.scatter(t[:,0],t[:,1],s=60,facecolors='none',edgecolors='red',label='Retained threat')
for a,b,i,j in zip(points[:-1],points[1:],ids[:-1],ids[1:]):
 if j==i+1: ax.plot([a[0],b[0]],[a[1],b[1]],color='orange',alpha=.4,lw=.6)
ax.set(xlim=(-.65,.4),ylim=(.25,.8),xlabel='body x (m)',ylabel='body y (m)',title='Sensor-only local correspondence at t=60.36 s')
ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend(fontsize=8);fig.tight_layout()
fig.savefig(root/'build/r86_surface_geometry.png',dpi=150)
(root/'build/r86_surface_geometry.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf8')
print(json.dumps([dict(previous_xy=r['previous_xy'],best=r['patches'][:1]) for r in rows]))
