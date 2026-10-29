"""완료된 새 주행의 실제 옵션·호출·정체만 회수한다. 기존 판정은 수정하지 않는다."""
from collections import Counter
import importlib.util
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
spec=importlib.util.spec_from_file_location('traffic_summary',BASE.parent/'g1_traffic_lap_v1/summarize_run.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
run=BASE/'run1_course';out=BASE/'run1_diagnostics.json'
if out.exists():raise FileExistsError(out)
result=module.summarize(run)
counts=Counter();selected=[];header=None
for line in (run/'follow_decisions.jsonl').open():
    row=json.loads(line)
    if row['kind']=='header':header=row['data'];continue
    if row['kind']!='call':continue
    data=row['data'];meta=data['output'][2];candidate=meta.get('corridor_candidate') or {}
    if candidate.get('short_prefix'):
        counts['short_proposal_calls']+=1
        counts['reason:'+str(candidate.get('reason'))]+=1
    if meta.get('boundary_selection')=='current_short_observed_prefix_candidate':
        counts['selected_calls']+=1
        counts['positive_final_commands']+=int(data['output'][0]>0)
        selected.append(dict(t_s=data['input']['reference_s'],command=data['output'][:2],
            selector_reason=meta.get('selector_reason'),corridor=candidate))
result.update(runtime_short_prefix_enabled=header['follow_config'].get('short_boundary_prefix_enabled'),
    short_prefix_counts=dict(counts),selected_short_prefix=selected)
out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('report_passed','run_valid','failed_gates','progress',
    'runtime_short_prefix_enabled','short_prefix_counts','stationary_spans_at_least_2p5s')},ensure_ascii=False))
