"""실제 r64 출력의 정지 사유를 분리한다. 추가 주행·제어기 변경 없음."""
from collections import Counter
import hashlib,json
from pathlib import Path

A=Path(__file__).resolve().parent


def main():
    path=A/'run1/follow_decisions.jsonl'
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    counts=Counter();windows=[]
    for line in path.open(encoding='utf-8'):
        event=json.loads(line)
        if event['kind']!='call':continue
        d=event['data'];speed,steer,detail=d['output'];t=d['input']['reference_s']
        n=detail.get('normal') or detail
        if speed>0:continue
        key=(n.get('selector_mode'),n.get('baseline_reason'),n.get('selector_reason'))
        counts[key]+=1
        if not windows or windows[-1]['key']!=list(key) or t-windows[-1]['end_s']>.10:
            windows.append(dict(key=list(key),start_s=t,end_s=t,calls=1,
                corridor_reason=(n.get('corridor_candidate') or {}).get('reason')))
        else:windows[-1]['end_s']=t;windows[-1]['calls']+=1
    summary=dict(input_sha256=digest,input_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest,
        counts=[dict(state=k[0],baseline=k[1],selector=k[2],calls=v) for k,v in counts.most_common()],
        windows=windows,scope='zero command decisions, not measured stationary time')
    with (A/'baseline_summary_reproducible.json').open('x') as stream:stream.write(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(input_unchanged=summary['input_unchanged'],counts=summary['counts'])))

if __name__=='__main__':main()
