from pathlib import Path
import json
HERE = Path(__file__).resolve().parent
run = HERE.parent / 'solo_full_lap_r457_v1' / 'run1'
with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
    header = json.loads(next(f))
print('HEADER',json.dumps({k:v for k,v in header['data'].items() if k in ('motion_config','follow_config')}))
rows=[json.loads(x) for x in (HERE/'saved_plan_window.jsonl').read_text().splitlines()]
for e in rows:
    d=e['data']
    if d['sequence'] not in (641,643,644):continue
    if e['kind']=='runtime_plan_submitted':
        print('INPUT',d['sequence'],json.dumps({k:(list(v) if isinstance(v,dict) else v) for k,v in d['input'].items() if k not in ('scan','history')}))
        print('HISTORY_KEYS',list(d['input'].get('history',{})))
    else:
        n=d['output'][2]['normal']
        print('NORMAL',d['sequence'],json.dumps({k:v for k,v in n.items() if k not in ('path_points','selector_evidence','selector_sides','partial_observation')},indent=2))
