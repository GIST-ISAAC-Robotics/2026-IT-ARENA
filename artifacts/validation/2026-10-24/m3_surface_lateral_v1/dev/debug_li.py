import math, sys, json
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
from arena_autonomy.lateral_intent import *
scene, path = straight(0., dropout=lambda deg: (deg >= 90) & (deg <= 150))
grid = ObservationGrid(scene, GRASS)
i, j, _ = grid.index(np.array([[-.6, 0.], [0., -.15], [0., .15], [.5, 0.]]))
print('states', [int(grid.state[a, b]) for a, b in zip(i, j)])
print('counts', {k: int(np.sum(grid.state == v)) for k, v in (('UNOBS', 0), ('FREE', 1), ('OCC', 2), ('EGO', 3))})
# rays status counts
from collections import Counter
print(Counter(r.status for r in scene.rays))
scene, path = straight(-.09)
lim = LateralIntentLimiter(GRASS)
out = lim.step(scene, path, .35)
print(out['mode'], out['reasons'])
for k, v in out['checks'].items():
    print(k, v['verified'], v['reasons'], {x: v.get(x) for x in ('new_cells', 'new_cells_unobserved', 'hidden_reach_cells', 'hidden_min_margin_s')}, v.get('surface'), v.get('lateral_clearance'))
print('seeds', out['hidden_seed_cells'])
