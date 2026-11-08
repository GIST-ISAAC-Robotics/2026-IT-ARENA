from test_moving_pass_faster import faster_controller
from test_moving_pass import history
from probe_bypass_entry import sensor_scene
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.bypass_handoff import front_patch
from arena_autonomy.moving_pass import available_pass_offsets
for width,y in [(.425,-.12),(.505,-.12),(.6,-.20),(.6,-.12),(.65,-.12)]:
 c=faster_controller(); s,_=sensor_scene(targets=((1.2,y),),wall_half_width=width)
 _,r=straight_reference(s,c.config);p,_=front_patch(s,'contiguous_three_v0')
 print(width,y,'patch',p is not None,'walls',r)
 if p is not None and r.get('walls'):
  for fixed in (True,False):print(fixed,available_pass_offsets(s,c.config,r['walls'],p,fixed_outer_candidate=fixed,maximum_offset=.35))
 print('out',c.moving_pass.guide.evaluate(s,history(),.7,0.,.02))
