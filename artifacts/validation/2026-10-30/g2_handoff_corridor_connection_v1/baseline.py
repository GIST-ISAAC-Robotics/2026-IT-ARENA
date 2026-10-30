import hashlib,json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
reports=[]
for run in ('run1','run2'):
 p=BASE.parent.parent/'2026-10-29/g2_handoff_gazebo_v1'/run/'g2_decisions.jsonl'
 events=[json.loads(line) for line in p.open()];h=events[0]['data']
 ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
 m=MotionConfig(**h['motion_config']);dig=hashlib.sha256();n=matched=0
 for e in events[1:]:
  if e['kind']=='reset':ctl.reset()
  elif e['kind']=='call':
   out=freeze(replay_call(ctl,e['data']['input'],m));n+=1;matched+=equal_output(out,e['data']['output'])
   dig.update(json.dumps(out,sort_keys=True,separators=(',',':')).encode())
 reports.append(dict(run=run,calls=n,historical_matched=matched,current_output_sha256=dig.hexdigest()))
tag=sys.argv[1]
result=dict(scope='current runtime output baseline; historical mismatches are preserved',reports=reports)
if tag=='after':
 before=json.loads((BASE/'before.json').read_text());result['unchanged']=before['reports']==reports
with (BASE/(tag+'.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
if tag=='after' and not result['unchanged']:raise SystemExit(1)
