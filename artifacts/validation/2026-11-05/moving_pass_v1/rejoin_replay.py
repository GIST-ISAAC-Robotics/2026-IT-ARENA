"""오른쪽 12 cm 실제 실패의 고정 입력에서 복귀 안내 기준만 교체한다."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import sys
from arena_autonomy.follow_trace import replay_events
from review import read,lines

HERE=Path(__file__).resolve().parent
def main(name):
    dest=HERE/(name+'.json')
    if dest.exists():raise FileExistsError(dest)
    base=HERE/'closed_loop_side_pass_1/run1'
    trace=base/'follow_decisions.jsonl';green=read(base/'report.json')['case_window']['green_sim_s']
    old={}
    def events():
        for event in lines(trace):
            if event['kind']=='call':
                output=event['data']['output']
                old[event['seq']]=(output[0],output[2].get('moving_pass',{}).get('entry',{}).get('reason'))
            yield event
    result=replay_events(events());counts=Counter();samples=[];prev=None;first=None
    for r in result['outcomes']:
        if old[r['seq']][1]!='entry_guide_current_wall_too_short':continue
        entry=r['details'].get('moving_pass',{}).get('entry',{})
        marked=any('guide_extent' in w for w in (entry.get('reference') or {}).get('walls',{}).values())
        key=(str(entry.get('reason')),bool(r['speed_mps']>0),marked)
        counts[str(key)]+=1
        if key!=prev:
            samples.append(dict(t=round(r['reference_s']-green,3),speed=r['speed_mps'],decision=key))
        if first is None and key[1] and marked:first=round(r['reference_s']-green,3)
        prev=key
    off=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/follow_decisions.jsonl'
    normal=replay_events(lines(off))
    values=dict(scope='stored poses not a closed-loop completion',input_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        calls=result['calls'],previous_short=sum(counts.values()),first_forward_rejoin_s=first,
        counts=dict(counts),samples=samples,off_calls=normal['calls'],off_mismatches=len(normal['mismatches']))
    with dest.open('x',encoding='utf-8') as f:json.dump(values,f,indent=1);f.write('\n')
    print({k:v for k,v in values.items() if k!='samples'})
    return first is not None and normal['passed']
if __name__=='__main__':sys.exit(0 if main(sys.argv[1]) else 1)
