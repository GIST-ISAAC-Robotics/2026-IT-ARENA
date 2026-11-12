import bisect,json,collections
from pathlib import Path
A=Path(__file__).resolve().parent

def audit(folder):
 rows=[json.loads(s)for s in (folder/'run1/follow_decisions.jsonl').open()]
 applied=[]
 for e in rows:
  if e['kind']!='call':continue
  d=e['data'];delivery=d.get('delivery')or{}
  if delivery.get('kind')!='candidate':continue
  first=d['input']['scan']['measurements']['acquired_at_s'][0]
  at=first+delivery['source_age_s']
  ref=d.get('current_rebase',{}).get('output',{}).get('reference_s',d['input']['reference_s'])
  n=d['output'][2].get('normal')or{}
  applied.append((at,ref,n.get('selector_mode'),'current_rebase'in d,e['seq']))
 applied.sort();times=[v[0]for v in applied]
 trace=[json.loads(s)for s in (folder/'run1/trajectory.jsonl').open()]
 groups=collections.Counter();matched=0
 for a,b in zip(trace,trace[1:]):
  if a.get('safety_reason')!='stale_input' or a.get('drive_speed_mps',0)<=0:continue
  i=bisect.bisect_right(times,a['t'])-1
  if i<0:continue
  t,ref,mode,rebase,seq=applied[i]
  age=a['t']-ref
  groups[(mode,rebase,age>.2+1e-8)]+=b['t']-a['t'];matched+=1
 return dict(stale_positive_samples=matched,intervals=[dict(selector_mode=k[0],current_rebase=k[1],inferred_command_age_exceeds_0p2=k[2],duration_s=v)for k,v in groups.most_common()],
  scope='inferred from recorded apply/source reference, not direct header measurement at safety callback; sensor-stop publications and callback skew can differ')

if __name__=='__main__':
 result={}
 for name in ('path_side_runtime_r165_v1','released_runtime_r168_v1'):
  f=A.parent/name/'two_actor_run1'
  if (f/'run1/report.json').exists():result[name]=audit(f)
 (A/'timing_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
