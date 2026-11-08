"""빠른 추월의 최초 편이 제안만 동적 외벽 여유와 맞춘 미연결 저장 입력 대조."""
from pathlib import Path
import hashlib,json
from collections import Counter
import numpy as np
import arena_autonomy.moving_pass as mp
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,freeze,replay_call,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
A=Path(__file__).resolve().parent
orig=mp.available_pass_offsets

def conservative(*args,**kwargs):
 # 입구 계산과 같은 광선 취득 나이/여유. 값을 낮추거나 가림을 자유로 채우지 않는다.
 kwargs['fixed_outer_candidate']=False
 return orig(*args,**kwargs)

traces={
 'r32': 'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl',
 'right20': 'faster_onroad_space_v2/straight_moving_lead/run1/follow_decisions.jsonl',
 'medium': 'medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl'}
allresults={}
for label,rel in traces.items():
 path=A.parent/rel;rows=[];recorded_mismatch=0
 for ln in path.open(encoding='utf8'):
  e=json.loads(ln);d=e['data']
  if e['kind']=='header':
   cfg=FollowConfig(**d['follow_config']);ex=HandoffExecutionConfig.from_mapping(d['execution_config']);mc=MotionConfig(**d['motion_config'])
   base=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True);candidate=BypassHandoffFollow(cfg,ex,allow_simulation_trial=True)
  elif e['kind']=='reset':base.reset();candidate.reset()
  elif e['kind']=='call':
   mp.available_pass_offsets=orig
   old=freeze(replay_call(base,d['input'],mc))
   mp.available_pass_offsets=conservative if ex.moving_pass_profile=='faster_v1' else orig
   try:new=freeze(replay_call(candidate,d['input'],mc))
   finally:mp.available_pass_offsets=orig
   recorded_mismatch+=not np.allclose(old[:2],d['output'][:2],rtol=0,atol=1e-10)
   rows.append(dict(t=d['input']['reference_s'],old=old[:2],new=new[:2],old_reason=old[2].get('reason'),new_reason=new[2].get('reason'),old_active=base.moving_pass.active,new_active=candidate.moving_pass.active,
    candidate_reason=new[2].get('moving_pass',{}).get('entry',{}).get('reason'),normal_reason=new[2].get('normal',{}).get('reason')))
 first=next((i for i,r in enumerate(rows) if r['old_active']),len(rows))
 # 비교 입력 자세는 원래 추월 궤적이다. 그 뒤 출력은 대안 실제 궤적이 아님.
 window=rows[max(0,first-10):min(len(rows),first+6)]
 summary=dict(calls=len(rows),baseline_command_mismatches=recorded_mismatch,
  changed_commands=sum(not np.allclose(r['old'],r['new'],rtol=0,atol=1e-10) for r in rows),
  first_original_entry_s=None if first==len(rows) else rows[first]['t'],entry_window=window,
  old_active=sum(r['old_active'] for r in rows),new_active=sum(r['new_active'] for r in rows),
  old_zero=sum(r['old'][0]==0 for r in rows),new_zero=sum(r['new'][0]==0 for r in rows),
  reasons=dict(Counter(r['new_reason'] for r in rows)),
  source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
 allresults[label]=summary
 with (A/(label+'_probe_rows.jsonl')).open('x',encoding='utf8') as f:
  for r in rows:f.write(json.dumps(r)+'\n')
with (A/'probe_summary.json').open('x',encoding='utf8') as f:json.dump(allresults,f,indent=2)
print(json.dumps({k:{kk:vv for kk,vv in v.items() if kk!='entry_window'} for k,v in allresults.items()}))
