"""중앙 정체의 저장 입력에서 실제 낮은 명령 후보를 같은 판단 직전 상태로 대조한다."""
import copy,json
from dataclasses import replace
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from candidate import CorridorHandoff
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

def main():
 path=BASE/'candidate2/center_combined__observed_entry_space_v1.jsonl'
 events=[json.loads(line) for line in path.open()];h=events[0]['data']
 ctl=CorridorHandoff(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
 motion=MotionConfig(**h['motion_config']);reports=[];n=matched=0
 for e in events[1:]:
  if e['kind']=='reset':ctl.reset()
  elif e['kind']=='call':
   d=e['data'];inp=d['input'];t=inp['reference_s']
   target=any(abs(t-w)<1e-8 for w in (7.58,9.,15.))
   before=copy.deepcopy(ctl) if target else None
   result=replay_call(ctl,inp,motion);n+=1;matched+=equal_output(freeze(result),d['output'])
   if not target:continue
   row=dict(t=t,measured_speed=inp['speed_mps'],primary=result[:2],primary_reason=result[2].get('bypass',{}).get('reason'),alternatives=[])
   for cap in (.25,.15,.08):
    trial=copy.deepcopy(before);g=trial.bypass.guide
    g.entry=replace(g.entry,speed_cap_mps=cap);g.registered_entry=replace(g.registered_entry,speed_cap_mps=cap)
    out=replay_call(trial,inp,motion);bg=out[2].get('bypass') or {}
    row['alternatives'].append(dict(cap=cap,command=out[:2],reason=bg.get('reason'),phase=bg.get('entry_phase'),
      candidates=[{k:r.get(k) for k in ('reason','motion_region','target_speed_mps','command_clearance_m')} for r in bg.get('candidates',[])]))
   reports.append(row)
 result=dict(scope='same pre-call state recorded input; no new driving',calls=n,matched=matched,reports=reports)
 with (BASE/'slow_shadow1.json').open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result));return 0 if n==matched and len(reports)==3 else 1
if __name__=='__main__':raise SystemExit(main())
