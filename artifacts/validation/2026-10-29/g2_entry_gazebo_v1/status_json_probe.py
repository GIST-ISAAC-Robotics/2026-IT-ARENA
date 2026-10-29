"""G2 판단 detail이 기존 WallFollow 상태 발행(json.dumps allow_nan=False)을 통과하는지 재현한다. 수정 전 진단."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[4]
for d in ('src/arena_autonomy','src/arena_vehicle_interface','scripts','tests'):sys.path.insert(0,str(ROOT/d))
from test_bypass_entry_follow import inputs
from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

follow=BypassEntryFollow(FollowConfig(),BypassExecutionConfig(),allow_simulation_trial=True)
rows=[]
for i,now in enumerate((.5,.52,.6,.62,.7,.72)):
    scan,history=inputs(.5 if i<2 else .6 if i<4 else .7)
    scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
    target,steering,detail=follow.command(scene,history,0.,'left',0.,.02)
    status={'state':'X','sim_time_s':now};status.update(detail)
    try:
        payload=json.dumps(status,ensure_ascii=False,allow_nan=False);rows.append(dict(call=i,ok=True,bytes=len(payload),reason=detail.get('reason')))
    except (TypeError,ValueError) as error:
        rows.append(dict(call=i,ok=False,error=f'{type(error).__name__}: {error}'[:200],reason=detail.get('reason'),
                         candidates=len(detail.get('candidates',[]))))
print(json.dumps(rows,ensure_ascii=False,indent=1))
