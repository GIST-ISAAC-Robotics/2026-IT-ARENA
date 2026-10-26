import sys
sys.path[:0]=['tests']
from dataclasses import replace
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import car, box
from test_local_follow import history
from test_m3_execution_trial import ctl
from test_m3_observed_follow import timed_scene
ON = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True, small_center_straight=True)
OFF = replace(ON, small_center_straight=False)
D0=-.004
def seq(lead_x):
    out=[(round(.4+.1*k,3), []) for k in range(3)]
    for k in range(16):
        t=round(.7+.1*k,3)
        extra=[car(.30+.045*k, .26, D0)] if k<5 else []
        if lead_x is not None and k>=6:
            extra=extra+[car(lead_x, 0., D0)]
        out.append((t, extra))
    return out
for lead_x in (None, .60, .45):
    for name, ex in (('OFF',OFF),('ON',ON)):
        f=ctl(ex); h=history(v=0.)
        rows=[]
        for t, extra in seq(lead_x):
            scene,_=timed_scene(t, D0, extra=extra)
            o=f.command(scene,h,0.,'left')
            lat=o[2].get('m3_lateral') or {}
            rows.append((t, round(o[0],3), round(o[1],4), o[2]['reason'], lat.get('mode'), bool((lat.get('small_center') or {}).get('ok')), round(o[2].get('swept_clearance_m') or -1,3)))
        print(lead_x, name)
        for r in rows[3:]: print('   ', r)
