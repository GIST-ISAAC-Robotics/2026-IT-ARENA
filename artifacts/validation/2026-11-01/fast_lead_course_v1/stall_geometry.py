"""저장 입력의 정체 시점에서 거절 영역과 실제 광선 상태를 분리한다."""
from pathlib import Path
import json,math
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import motion_region,lateral_config
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn

BASE=Path(__file__).resolve().parent
header=json.loads((BASE/'run1/follow_decisions.jsonl').open().readline())['data']
sample=json.loads((BASE/'stall_input30.json').read_bytes())
data,scan,history=restore_call(sample['input'],MotionConfig(**header['motion_config']))
scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
    epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
c=FollowConfig(**header['follow_config']);lat=lateral_config(c);context={}
target=.35;steer=sample['output'][1]
region=motion_region(scene,data['speed_mps'],target,steer,data['previous_steering_rad'],data['dt_s'],c,lat,context=context)
queries=context['queries'][~context['free'] & ~context['masked']]
diagnostic=dict(accepted=False,reason='diagnostic',unknown_filled=False,residual_hidden_object_risk=True,
    maximum_gap_rad=math.radians(3),minimum_neighbor_range_m=.6)
neighbors=_bounded_nonreturn(scene,scan,queries,context['padding'],diagnostic)
hits=np.array([r.hit_xy_m for r in scene.rays if r.status=='return'])
closest=float(np.min(np.linalg.norm(queries[:,None,:]-hits[None,:,:],axis=2)))
answer=dict(reference_s=data['reference_s'],speed=data['speed_mps'],target=target,steer=steer,
    previous=data['previous_steering_rad'],region=region,unknown_query_min=queries.min(axis=0).tolist(),
    unknown_query_max=queries.max(axis=0).tolist(),queries=queries.tolist(),nonreturn_only=neighbors,
    closest_known_hit_m=closest,
    scope='원래 1 mm 외곽 제한만 분리한 진단. 연결된 허용 조건이나 실제 주행 변경 아님.')
with (BASE/'stall_geometry.json').open('x',encoding='utf-8') as f:json.dump(answer,f,ensure_ascii=False,indent=2)
print(json.dumps({k:v for k,v in answer.items() if k!='queries'},ensure_ascii=False))
