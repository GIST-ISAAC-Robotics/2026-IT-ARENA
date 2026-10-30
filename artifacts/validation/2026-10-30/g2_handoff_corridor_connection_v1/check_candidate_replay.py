from dataclasses import replace
import json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,replay_events
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
reports=[]
for p in sorted((BASE.parent/'g2_handoff_corridor_v1/candidate3').glob('*.jsonl')):
 events=[json.loads(line) for line in p.open()];h=events[0]['data']
 cfg=replace(HandoffExecutionConfig.from_mapping(h['execution_config']),entry_policy='corridor_local_v1')
 ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),cfg,allow_simulation_trial=True)
 motion=MotionConfig(**h['motion_config']);n=matched=0;first=[]
 for e in events[1:]:
  if e['kind']=='reset':ctl.reset()
  elif e['kind']=='call':
   d=e['data'];out=freeze(replay_call(ctl,d['input'],motion));n+=1
   eq=equal_output(out,d['output']);matched+=eq
   if not eq and len(first)<2:first.append(dict(t=d['input']['reference_s'],before=d['output'][:2],after=out[:2],before_reason=d['output'][2].get('reason'),after_reason=out[2].get('reason')))
 # This revised header is an explicitly selected adapter for these saved inputs, not a past runtime execution.
 h['execution_config']['entry_policy']='corridor_local_v1'
 schema=replay_events(events)
 reports.append(dict(trace=p.name,calls=n,matched=matched,first_mismatches=first,schema5_passed=schema['passed'],schema5_mismatches=len(schema['mismatches'])))
result=dict(scope='offline candidate outputs against current explicit runtime and schema5 adapter; no new drive',reports=reports,passed=all(r['calls']==r['matched'] and r['schema5_passed'] for r in reports))
with (BASE/'candidate_replay1.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
