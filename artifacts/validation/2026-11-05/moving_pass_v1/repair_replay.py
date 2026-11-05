"""첫 실제 추월 실패의 고정 입력. 달라진 명령과 여전히 멈추는 원인을 나눈다."""
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
    base=HERE/'closed_loop_v2/run1'
    trace=base/'follow_decisions.jsonl'
    tg=read(base/'report.json')['case_window']['green_sim_s']
    before={}
    def events():
        for e in lines(trace):
            if e['kind']=='call':
                d=e['data']['output'];before[e['seq']]=(d[0],d[2].get('moving_pass',{}).get('entry',{}).get('reason'))
            yield e
    res=replay_events(events())
    reasons=Counter();changes=Counter();samples=[];last=None;witness=0
    for r in res['outcomes']:
        old_speed,old_reason=before[r['seq']]
        if old_reason!='entry_guide_current_wall_too_short':continue
        entry=r['details'].get('moving_pass',{}).get('entry',{})
        selected=entry.get('selected');candidates=entry.get('candidates',[])
        reason=entry.get('reason')
        why=tuple(c.get('reason') for c in candidates)
        witnessed=any('rear_geometry_witness' in w for w in (entry.get('reference') or {}).get('walls',{}).values())
        witness+=int(witnessed);reasons[(str(reason),','.join(str(v) for v in why))]+=1
        changed=(r['speed_mps']>0, witnessed)
        changes[str(changed)]+=1
        key=(reason,why,changed)
        if last!=key:
            samples.append(dict(time=round(r['reference_s']-tg,3),seq=r['seq'],speed=r['speed_mps'],
                reason=reason,first_candidate=next(iter(candidates),None),witness=witnessed))
        last=key
    off=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1/follow_decisions.jsonl'
    off_res=replay_events(lines(off))
    result=dict(scope='stored_pose_fixed_sensor_input_not_closed_loop',source=trace.as_posix().split('/moving_pass_v1/')[-1],
        input_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),calls=res['calls'],previous_short=sum(reasons.values()),
        current_reasons={' / '.join(k):v for k,v in reasons.items()},positive_and_witness=changes,witness=witness,
        samples=samples,off_calls=off_res['calls'],off_mismatches=len(off_res['mismatches']))
    with dest.open('x',encoding='utf-8') as f:json.dump(result,f,indent=1,ensure_ascii=False);f.write('\n')
    print({k:v for k,v in result.items() if k!='samples'})
    if not off_res['passed']:raise SystemExit(1)
if __name__=='__main__':main(sys.argv[1])
