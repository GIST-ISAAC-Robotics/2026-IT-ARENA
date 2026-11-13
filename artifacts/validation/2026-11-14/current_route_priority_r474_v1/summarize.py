"""기존 재생 산출물만 집계한다. evaluator/ROS 실행 없음."""
from pathlib import Path
import copy,json,statistics,hashlib
A=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
rows=[json.loads(s) for s in (A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl').open(encoding='utf-8')]
calls={r['seq']:r['data'] for r in rows if r['kind']=='call'}
legacy=load(A/'replay_v2/legacy_current_rows.json')
normalized=[]
for row in legacy:
    expected=copy.deepcopy(calls[row['trace_seq']]['current_rebase']['output'])
    cap=expected.pop('new_plan_deceleration_cap',None)
    if cap:expected['speed_mps']=cap['checked_target_mps']
    normalized.append(dict(trace_seq=row['trace_seq'],raw_exact=row['exact'],
        recorded_parent_cap=cap,child_comparison_exact=expected==row['output']))
timing={}
logged=[(r['data']['completed_wall_s']-r['data']['began_wall_s'])*1000
    for r in rows if r['kind']=='runtime_plan_completed']
timing['r469_original_live_logged']=dict(n=len(logged),median_ms=statistics.median(logged),max_ms=max(logged),first_ms=logged[0],
    scope='original live simulation worker timestamps; different native source/load, not paired speedup evidence')
for version in ('replay_v1','replay_v2'):
    plans=load(A/version/'plan_rows.json');ms=[r['ms'] for r in plans]
    timing[version]=dict(n=len(ms),median_ms=statistics.median(ms),max_ms=max(ms),first_ms=ms[0],
        scope='existing saved-plan evaluator wall measurement on WSL; not ROS/Jetson runtime')
comparisons=load(A.parent/'solo_stall_diagnostics_r472_v1/diagnostic_v1/comparisons.json')
if isinstance(comparisons,list) and comparisons and all('ms' in r for r in comparisons):
    ms=[r['ms'] for r in comparisons]
    timing['r472_frozen_r469']=dict(n=len(ms),median_ms=statistics.median(ms),max_ms=max(ms),first_ms=ms[0],
        scope='previous separate WSL diagnostic, same 217 saved plans; not paired benchmark')
result=dict(legacy_raw_exact=sum(r['raw_exact'] for r in normalized),legacy_total=len(normalized),
    legacy_child_comparison_exact=sum(r['child_comparison_exact'] for r in normalized),
    raw_differences=[r for r in normalized if not r['raw_exact']],timing=timing)
(A/'comparison_details.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='raw_differences'}))
assert result['legacy_child_comparison_exact']==113
