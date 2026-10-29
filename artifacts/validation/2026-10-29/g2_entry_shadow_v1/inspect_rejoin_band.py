"""복귀 관측 띠의 미관측 위치 진단. 주행 시험 아님."""
import json
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryGuide,observed_straight_wall
from arena_autonomy.observation_support import depth_support
from arena_autonomy.pass_release_trial import lateral_config
import arena_autonomy.m3_execution_trial as mx
from test_bypass_entry_guide import scene_at_side
for xpos in (1.2,1.3,1.4,2.):
    scene=scene_at_side(xpos);guide=EntryGuide(rejoin_enabled=True);guide.side='left';guide.offset=.16
    fit,_=observed_straight_wall(scene,guide.config,'left')
    slope,intercept,_,_=fit;offset=abs(intercept)/np.sqrt(1+slope*slope)
    x=np.arange(-.1,.501,.02);y=-np.arange(.09,offset+.101,.02)
    q=np.stack(np.meshgrid(x,y,indexing='ij'),axis=-1).reshape(-1,2)
    pad=mx.ray_padding(scene,lateral_config(guide.config),.5)
    free=depth_support(scene,q,pad);masked=mx.masked_only(scene,q[~free],pad)
    print(json.dumps(dict(x=xpos,queries=len(q),unobserved=int((~free).sum()),masked=int(masked.sum()),
        unsupported=q[~free].tolist(),mid_front_supported=bool(np.all(free[q[:,0]>=-1e-9])))))
