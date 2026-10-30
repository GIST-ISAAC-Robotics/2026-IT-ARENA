from collections import Counter
import hashlib,json
from pathlib import Path
import sys
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import g2_handoff_eval_v3 as ev
output=BASE/'run1';config=json.loads((BASE/'g2_center_config_v1.json').read_text())
result=ev.evaluate(output,config)
old=json.loads((BASE/'run1_g2_evaluation.json').read_text())
rows=[]
for line in (output/'g2_decisions.jsonl').open():
 event=json.loads(line)
 if event['kind']=='call':
  d=event['data'];t=d['input']['reference_s']
  if result['green_sim_s']<=t<=result['window_end_sim_s']:
   out=d['output'];detail=out[2]
   rows.append(dict(t=t,speed=out[0],state=detail.get('handoff_state'),reason=detail.get('reason'),
     normal_mode=(detail.get('normal') or {}).get('mode'),normal_reason=(detail.get('normal') or {}).get('reason')))
transitions=[]
for row in rows:
 if not transitions or transitions[-1]['state']!=row['state']:
  transitions.append(dict(row,since_green_s=row['t']-result['green_sim_s']))
cycle=[r['state'] for r in transitions]
returned=next((r for r in rows if r['reason']=='handoff_return'),None)
after=[r for r in rows if returned is not None and r['t']>returned['t'] and r['state']=='FOLLOW']
checks=dict(automatic_cycle=cycle[:4]==['FOLLOW','BYPASS','REACQUIRE','FOLLOW'],
 explicit_g1_return=returned is not None and returned['normal_mode']=='g1_follow',
 g1_output_after_return=bool(after) and any(r['speed']>0 for r in after),
 bypass_reacquire_cap=all(0<=r['speed']<=.35+1e-9 for r in rows if r['state'] in ('BYPASS','REACQUIRE')))
changed={k:[old['gates'].get(k),v] for k,v in result['gates'].items() if old['gates'].get(k)!=v}
checks['only_original_mode_gate_changed']=changed=={'g2_controller_rows':[False,True]}
result['gates'].update(checks)
result['passed']=all(result['gates'].values()) and all(result['run_checks'].values())
result['handoff_audit']=dict(transitions=transitions,after_return_calls=len(after),
 state_counts=dict(Counter(r['state'] for r in rows)),changes_to_original_gates=changed)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result['reevaluation']=dict(scope='saved run only, original failed run/evaluator preserved, no new drive',
 original_result_sha256=sha(BASE/'run1_g2_evaluation.json'),
 source_sha256={p.name:sha(p) for p in (BASE/'g2_handoff_eval_v3.py',BASE/'g2_handoff_eval_core_v3.py',BASE/'reevaluate.py')})
with (BASE/'run1_evaluation_v3.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(dict(passed=result['passed'],checks=checks,transitions=transitions,after_return_calls=len(after),
 failed=[k for k,v in result['gates'].items() if not v]),ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 1)
