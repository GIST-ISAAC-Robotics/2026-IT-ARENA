"""첫 플랫폼 차이 호출의 실제 운동 영역 좌표를 보존한다. 출력 변경 없음."""
import argparse
import json
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call, freeze
import arena_autonomy.pass_release_trial as ps

p=argparse.ArgumentParser();p.add_argument('trace',type=Path);p.add_argument('--output',type=Path,required=True)
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
events=[json.loads(line) for line in args.trace.open()]
header=events[0]['data']
assert header['scope']=='g1_g2_handoff_decision_stage_only'
owner=BypassHandoffFollow(FollowConfig(**header['follow_config']),
    HandoffExecutionConfig.from_mapping(header['execution_config']),allow_simulation_trial=True)
motion=MotionConfig(**header['motion_config']);rows=[];arrays={};capture=False
original=ps.motion_queries
def observed(*a,**kw):
    distance,q=original(*a,**kw)
    if capture:
        key=f'q{len(rows)}';arrays[key]=q
        rows.append(dict(key=key,distance=distance,queries=len(q),speed=a[1],target=a[2],
                         steering=a[3],previous=a[4],dt=a[5]))
    return distance,q
ps.motion_queries=observed
try:
    for event in events[1:]:
        if event['seq']>401:break
        if event['kind']=='reset':owner.reset()
        if event['kind']=='call':
            capture=event['seq']==401
            output=freeze(replay_call(owner,event['data']['input'],motion))
finally:ps.motion_queries=original
np.savez_compressed(args.output/'queries.npz',**arrays)
(args.output/'geometry.json').write_text(json.dumps(dict(seq=401,rows=rows,output=output),indent=2)+'\n')
print(json.dumps(rows))
