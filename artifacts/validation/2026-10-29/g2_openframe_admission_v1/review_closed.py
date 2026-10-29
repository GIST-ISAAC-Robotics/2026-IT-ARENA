"""완료된 형상 비교의 진행/전이/점유 거절을 모은다. 실행 중인 파일은 읽지 않는다."""
from pathlib import Path
from collections import Counter
import json
BASE=Path(__file__).resolve().parent

def summarize(folder):
    report=json.loads((folder/'report.json').read_text())
    results=[]
    for case in report['cases']:
        name=case['case']+'__'+case['policy']
        counts=Counter(); failures=Counter();selected=[]
        with (folder/(name+'.jsonl')).open() as f:
            for line in f:
                e=json.loads(line)
                if e['kind']!='call':continue
                d=e['data']['output'][2];b=d.get('bypass',{})
                counts[b.get('reason','not_evaluated')]+=1
                if e['data']['output'][0]==0:
                    for c in b.get('candidates',[]):failures[c.get('reason','no_reason')]+=1
                if d.get('reason')=='handoff_entry':selected.append(b.get('selected'))
        results.append(dict(case=case['case'],progress=case['final_progress_m'],entry=case['entry'],
            returned=case['returned_to_follow'],contacts=case['contact_samples'],grass=case['max_wheels_touching_grass'],
            stopped=case['stopped'],selected=selected,bypass_reasons=dict(counts),zero_candidate_reasons=dict(failures),
            replay=case['schema5_replay']))
    return dict(source_unchanged=report['source_unchanged'],cases=results)

if __name__=='__main__':
    result={n:summarize(BASE/n) for n in ('closed1','closed_sparse1')}
    out=BASE/'closed_comparison.json'
    with out.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
