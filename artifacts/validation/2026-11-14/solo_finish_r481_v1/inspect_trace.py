"""r477 원 trace에서 plan/current 원인과 실제 입력을 읽어 독립 보존한다."""
import collections
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
BASE=A.parent/'solo_variations_r477_v1/cases/start_left20_yaw_plus3'
TRACE=BASE/'run1/follow_decisions.jsonl'


def main():
    plans={};completed={};calls=[];publications=[];header=None;counts=collections.Counter()
    for line,text in enumerate(TRACE.open(encoding='utf-8'),1):
        row=json.loads(text);d=row['data'];kind=row['kind'];counts[kind]+=1
        if kind=='runtime_evaluator_header':header=d['header']
        elif kind=='runtime_plan_submitted':plans[(d['generation'],d['sequence'])]=dict(line=line,trace_seq=row['seq'],data=d)
        elif kind=='runtime_plan_completed':completed[(d['generation'],d['sequence'])]=dict(line=line,trace_seq=row['seq'],data=d)
        elif kind=='call':calls.append(dict(line=line,trace_seq=row['seq'],data=d))
        elif kind=='route_publication':publications.append(dict(line=line,trace_seq=row['seq'],data=d))
    index=[];focused=[]
    for key,p in plans.items():
        d=p['data'];o=completed[key]['data']['output'];n=o[2].get('normal') or {}
        r=dict(key=list(key),line=p['line'],completed_line=completed[key]['line'],reference_s=d['input']['reference_s'],
            speed=o[0],steering=o[1],reason=o[2].get('reason'),handoff=o[2].get('handoff_state'),
            normal_reason=n.get('reason'),selector_reason=n.get('selector_reason'),selector_mode=n.get('selector_mode'),
            path_source_s=n.get('path_source_s'),boundary_from_memory=n.get('boundary_from_memory'))
        index.append(r)
        if d['input']['reference_s']>=49.:
            focused.append(dict(key=list(key),submitted=p,completed=completed[key]))
    for name,value in [('header',header),('plan_index',index),('focus_plans',focused),('kind_counts',dict(counts))]:
        (A/(name+'.json')).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    focus_calls=[]
    for r in calls:
        d=r['data'];ref=d.get('input',{}).get('reference_s',d.get('input',{}).get('now',0))
        if ref>=49:focus_calls.append(r)
    (A/'focus_calls.json').write_text(json.dumps(focus_calls,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (A/'publications.json').write_text(json.dumps(publications,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    (A/'input_source.json').write_text(json.dumps(dict(trace=str(TRACE),sha256=digest,plan_count=len(index),
        focus_plan_count=len(focused),focus_call_count=len(focus_calls),extraction_only=True),indent=2)+'\n')
    transitions=[];previous=None
    for r in index:
        if r['reference_s']<49:continue
        key=r['speed'],r['reason'],r['normal_reason'],r['selector_reason'],r['selector_mode']
        if key!=previous:transitions.append(r);previous=key
    print(json.dumps(dict(counts=dict(counts),focus_transitions=transitions,final_plan=index[-1],
        last_call_keys=list(calls[-1]['data']),last_call_input_keys=list(calls[-1]['data'].get('input',{}))),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
