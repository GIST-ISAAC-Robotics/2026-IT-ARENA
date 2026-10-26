import sys
sys.path[:0]=['tests']
from test_m3_small_center_straight import *
for d0 in (0., -.002, -.004, .004, -.0075):
    off, seen, _ = step_order(OFF, d0)
    print(d0, seen, {k:(v['eligible_for_trial'], v['reasons'], v.get('problems')) for k,v in off['checks'].items()})
