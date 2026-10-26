import sys
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
from arena_autonomy.lateral_intent import *
scene, path = straight(-.09)
cfg = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', hidden_relative_speed_mps=.1)
out, lim = run(scene, path, cfg, speed=.35)
print(out['mode'], out['reasons'])
for k, v in out['checks'].items():
    print(k, v['verified'], v['verified_except_hidden'], v['reasons'], v['problems'], {x: v.get(x) for x in ('new_cells', 'new_cells_unobserved', 'hidden_reach_cells')})
