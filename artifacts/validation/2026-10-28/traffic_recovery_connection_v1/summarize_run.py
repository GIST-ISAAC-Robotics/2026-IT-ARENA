"""실행 완료 뒤 교통 회복의 사용 근거와 진행을 회수한다. 판정을 변경하지 않는다."""
from collections import Counter
import importlib.util
import json
from pathlib import Path
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
source=ROOT/'artifacts/validation/2026-10-27/g1_traffic_lap_v1/summarize_run.py'
spec=importlib.util.spec_from_file_location('traffic_summary',source);summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)
run=BASE/'run1_course';out=BASE/'run1_diagnostics.json'
if out.exists():raise FileExistsError(out)
result=summary.summarize(run);counts=Counter();events=[];header=None
for line in (run/'follow_decisions.jsonl').open():
 row=json.loads(line)
 if row['kind']=='header':header=row['data'];continue
 if row['kind']!='call':continue
 d=row['data'];meta=d['output'][2];p=meta.get('partial_observation') or {};anchor=p.get('anchor_diagnostic') or {}
 if p.get('timed_missing_diagnostic'):counts['timed_geometry_calls']+=1
 if anchor:counts['anchor:'+str(anchor.get('reason'))]+=1
 if anchor.get('released'):
  counts['anchor_release_calls']+=1;events.append(dict(t_s=d['input']['reference_s'],kind='anchor_release',command=d['output'][:2],evidence=anchor))
 same=meta.get('same_path_gap') or (meta.get('selector_guard_candidate') or {}).get('same_path_gap') or {}
 if same.get('accepted'):counts['same_path_accepted_calls']+=1;counts['same_path_positive_calls']+=int(d['output'][0]>0)
 if meta.get('retained_passage'):counts['retained_passage_calls']+=1
result.update(runtime_traffic_recovery_enabled=header['follow_config'].get('traffic_recovery_trial'),recovery_counts=dict(counts),recovery_events=events)
out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('report_passed','run_valid','failed_gates','progress','runtime_traffic_recovery_enabled','recovery_counts','stationary_spans_at_least_2p5s')},ensure_ascii=False))
