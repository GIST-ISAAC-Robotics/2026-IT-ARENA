"""평가 전용 진행성: 녹색~STOP 구간의 5cm 이내 무진행 지속을 별도 보고한다."""
import argparse
from collections import deque,Counter
import hashlib
import json
from pathlib import Path


def episodes(rows,span_m=.05,minimum_s=2.5):
    lo=0;low=deque();high=deque();windows=[]
    for hi,row in enumerate(rows):
        value=row['progress_m']
        while low and rows[low[-1]]['progress_m']>=value:low.pop()
        while high and rows[high[-1]]['progress_m']<=value:high.pop()
        low.append(hi);high.append(hi)
        while rows[high[0]]['progress_m']-rows[low[0]]['progress_m']>span_m:
            lo+=1
            while low[0]<lo:low.popleft()
            while high[0]<lo:high.popleft()
        duration=row['t']-rows[lo]['t']
        if duration>=minimum_s:windows.append((duration,lo,hi))
    chosen=[]
    for duration,first,last in sorted(windows,reverse=True):
        if any(first<=e['_last'] and last>=e['_first'] for e in chosen):continue
        part=rows[first:last+1]
        chosen.append(dict(_first=first,_last=last,start_s=part[0]['t'],end_s=part[-1]['t'],duration_s=duration,
            progress_span_m=max(x['progress_m'] for x in part)-min(x['progress_m'] for x in part),
            start_progress_m=part[0]['progress_m'],end_progress_m=part[-1]['progress_m'],
            samples=len(part),positive_safe_samples=sum(x.get('safe_speed_mps',0)>0 for x in part),
            safety_reasons=dict(Counter(x.get('safety_reason') for x in part))))
    return [{k:v for k,v in item.items() if not k.startswith('_')} for item in chosen]


def assess(run):
    names=('report.json','trajectory.jsonl')
    hashes={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in names}
    report=json.loads((run/'report.json').read_text(encoding='utf-8'))
    first,last=report['green_observed_sim_s'],report['stop_request_sim_s']
    rows=[r for line in (run/'trajectory.jsonl').open(encoding='utf-8')
          if first<=(r:=json.loads(line))['t']<=last]
    found=episodes(rows)
    maximum=max((x['duration_s'] for x in found),default=0.)
    complete=bool(rows) and rows[0]['t']-first<.25 and last-rows[-1]['t']<.25
    checks=dict(coverage=complete,ordered=all(b['t']>a['t'] for a,b in zip(rows,rows[1:])),
                maximum_nonprogress_below_10s=maximum<10.,
                raw_unchanged=hashes=={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in names})
    return dict(passed=all(checks.values()),checks=checks,episodes=found,maximum_nonprogress_s=maximum,
        scope='registered follow-up progress criterion, truth-only evaluation; original evaluation preserved',
        policy=dict(span_m=.05,maximum_duration_s=10.,reason='representative basic drive without long immobilization; not an official penalty rule'),
        first_s=first,last_s=last,raw_sha256=hashes)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=assess(args.run)
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False));raise SystemExit(0 if result['passed'] else 1)
