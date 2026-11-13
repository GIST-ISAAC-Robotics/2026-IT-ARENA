"""Windows NumPy에서 원 helper를 한 번 분해한다. ARM 시간/전체 evaluator가 아니다."""
import cProfile
import json
from pathlib import Path
import pstats
import sys

A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'bundle/stage/src/arena_autonomy'),str(A/'bundle/stage/src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.known_structure import current_mask_shadow,parse_sectors

data=json.loads((A/'bundle/inputs.json').read_bytes())
record=next(r for r in data['records'] if r['id']=='r457:1590')
header=data['headers'][record['dataset']]
cfg=FollowConfig(**header['follow_config'])
info,scan,history=restore_call(record['current'],MotionConfig(**header['motion_config']))
scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
    epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
context={}
region=ps.motion_region(scene,max(0,record['current']['speed_mps']),.75,
    record['expected']['steering_rad'],record['current']['previous_steering_rad'],
    record['current']['dt_s'],cfg,ps.lateral_config(cfg),context=context)
profile=cProfile.Profile()
profile.enable()
result=current_mask_shadow(scene,scan,context['queries'][context['masked']],
    context['padding'],parse_sectors(cfg.known_structure_sectors_json))
profile.disable()
with (A/'local_helper_profile.txt').open('w') as out:
    out.write(json.dumps(dict(id=record['id'],region=region,result=result))+'\n')
    pstats.Stats(profile,stream=out).sort_stats('cumtime').print_stats(18)
print((A/'local_helper_profile.txt').read_text())
