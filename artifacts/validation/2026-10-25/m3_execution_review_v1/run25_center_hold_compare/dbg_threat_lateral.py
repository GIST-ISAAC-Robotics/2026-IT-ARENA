import sys
sys.path[:0]=['tests']
from test_m3_small_center_straight import *
for lat in (.30, .28, .26, .24, .22, .20):
    trial, outs = run_trial(ON, passing(-.004, lateral=lat), -.004)
    print(lat, [ (o['threats']['left']['points'], o['threats']['left']['extended_surface_points'], o['mode'], bool(o.get('small_center')), (o.get('release_gate') or {}).get('reason')) for o in outs])
