"""원 strict 결과와 명시된 표면 비용 진단만 제외한 결과를 함께 보존한다."""
import json
import sys
from collections import Counter
from pathlib import Path

A = Path(__file__).resolve().parent
sys.path[:0]=[str(A/'pydeps'),str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import comparable_decision


def differences(a,b,path='$'):
    if type(a) is not type(b):
        return [dict(path=path, before=a, after=b)]
    if isinstance(a,dict):
        if a.keys()!=b.keys():return [dict(path=path, before_keys=list(a), after_keys=list(b))]
        return [d for k in a for d in differences(a[k],b[k],path+'.'+k)]
    if isinstance(a,list):
        if len(a)!=len(b):return [dict(path=path,before_length=len(a),after_length=len(b))]
        return [d for i,(x,y) in enumerate(zip(a,b)) for d in differences(x,y,path+'['+str(i)+']')]
    return [] if a==b else [dict(path=path,before=a,after=b)]


def main():
    run=A/'windows_run_v3'
    rows=[]
    diff_counts=Counter();state_counts=Counter()
    with (run/'original/outputs.jsonl').open(encoding='utf8') as f, (run/'stage/outputs.jsonl').open(encoding='utf8') as g:
        for index,(left,right) in enumerate(zip(f,g,strict=True)):
            a,b=json.loads(left),json.loads(right)
            assert a['key']==b['key']
            strict=a['output']==b['output']
            diff=differences(a['output'],b['output']) if not strict else []
            state_diff=differences(a['state'],b['state']) if a['state']!=b['state'] else []
            diff_counts.update(d['path'] for d in diff);state_counts.update(d['path'] for d in state_diff)
            rows.append(dict(index=index,key=a['key'],command_exact=a['output'][:2]==b['output'][:2],
                strict_output_exact=strict,
                decision_exact=comparable_decision(a['output'])==comparable_decision(b['output']),
                strict_state_exact=not state_diff,output_differences=diff,state_differences=state_diff))
    before=json.loads((run/'original/summary.json').read_text())
    after=json.loads((run/'stage/summary.json').read_text())
    result=dict(count=len(rows),command_exact=sum(r['command_exact'] for r in rows),
        strict_output_exact=sum(r['strict_output_exact'] for r in rows),
        decision_exact=sum(r['decision_exact'] for r in rows),
        strict_state_exact=sum(r['strict_state_exact'] for r in rows),
        output_difference_paths=dict(diff_counts),state_difference_paths=dict(state_counts),
        view_calls_before=before['total_view_calls'],view_calls_after=after['total_view_calls'],
        hits=after['transfer_hits'],fallbacks=after['transfer_fallbacks'],
        slow_targets=[r for r in after['rows'] if r['key'][1] in (717,1304,1311)],
        original_header_preserved=True,derived_python_backend=True,native_used=False,
        input_sha256=after['input_sha256'],input_unchanged=before['input_unchanged'] and after['input_unchanged'],
        resets_exact=before['resets']==after['resets'],
        output_comparison='strict plus existing comparable_decision (only surface_correspondence/anchor_diagnostic.fit processing_ms)',
        state_comparison='all stored instance attributes/dataclass fields; callables by module/qualified name; cycles by stable path; no exclusions',
        rows=rows)
    (A/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
