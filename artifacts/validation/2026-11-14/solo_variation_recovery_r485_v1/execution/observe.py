import json,datetime
from pathlib import Path
import sys
artifact=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(artifact))
from prepare import CASES
assert len(sys.argv)==2 and sys.argv[1] in CASES
a=artifact/'cases'/sys.argv[1]
p=a/'run1/trajectory.jsonl'
if not p.exists():
 print(json.dumps(dict(phase='startup',trajectory_present=False)));raise SystemExit
rows=[]
for line in p.read_text(encoding='utf-8').splitlines():
 try:rows.append(json.loads(line))
 except json.JSONDecodeError:pass
if not rows:print('{}');raise SystemExit
green=[r for r in rows if r.get('elapsed_since_green_s') is not None]
last=rows[-1];maximum=None;last_advance=None
for r in green:
 if maximum is None or r['progress_m']>maximum+1e-6: maximum=r['progress_m'];last_advance=r['t']
idle=None if last_advance is None else last['t']-last_advance
contact=any(r.get('wall_collision') for r in green)
reason='wall_contact' if contact else 'eight_sim_seconds_no_progress' if idle is not None and idle>=8. else None
value=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),phase='running',rows=len(rows),last={k:last.get(k) for k in ['t','progress_m','ground_speed_mps','drive_speed_mps','safe_speed_mps','safety_reason','mcu_state','centerline_error_m','wall_collision']},last_progress_advance_s=last_advance,no_progress_sim_s=idle,stop_required=reason,report_present=(a/'run1/report.json').exists(),run_exit=(a/'execution_v1/run.exit').read_text().strip() if (a/'execution_v1/run.exit').exists() else None)
with (a/'execution_v1/observations.jsonl').open('a',encoding='utf-8') as stream:stream.write(json.dumps(value)+'\n')
print(json.dumps(value))
