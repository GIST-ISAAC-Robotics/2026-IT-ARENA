import math, sys
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
from arena_autonomy.lateral_intent import *
def show(out):
    print(out['mode'], out['reasons'], 'threats', {k: (v['present'], v['points'], v.get('nearest')) for k, v in out['threats'].items()})
    for k, v in out['checks'].items():
        print('  ', k, round(v['d_target_m'], 4), v['verified'], v['reasons'], {x: v.get(x) for x in ('new_cells', 'new_cells_unobserved', 'hidden_reach_cells', 'rear_zone_cells_excluded')}, v.get('surface', {}).get('reason'), v.get('lateral_clearance', {}).get('min_margin_m'), v.get('lateral_clearance', {}).get('worst'))
scene, path = straight(-.09)
out, _ = run(scene, path, speed=0.); show(out)
d0 = -.09
scene, path = straight(d0, extra=[car(-.40, .13, d0)])
out, _ = run(scene, path); show(out)
scene, path = straight(d0, extra=[car(-.40, .13, d0)], wall_gap_right=(-.4, .9))
out, _ = run(scene, path); show(out)
