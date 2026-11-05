"""새 포기 후보: 이미 성공한 저속 추월의 명령 유지와 가속 상대에서 복귀 후보를 나눈다."""
import sys
from collections import Counter
from prepare import HERE,save,read
from assess import rows
from arena_autonomy.follow_trace import replay_events

def replay(run):
    old={};window=read(run/'report.json')['case_window'];green=window['green_sim_s']
    def events():
        for event in rows(run/'follow_decisions.jsonl'):
            if event['kind']=='call':old[event['seq']]=event['data']['output'][:2]
            yield event
    result=replay_events(events());changed=[];counts=Counter();samples=[];last=None
    for row in result['outcomes']:
        t=row['reference_s']-green;d=row['details'];m=d.get('moving_pass',{})
        now=[row['speed_mps'],row['steering_rad']]
        if any(abs(a-b)>1e-9 for a,b in zip(now,old[row['seq']])):changed.append(row['seq'])
        if not 0<=t<=window['duration_s']:continue
        reason=d.get('reason');entry=m.get('entry',{});target=m.get('target',{})
        key=(reason,entry.get('reason'))
        counts[str(key)]+=1
        if key!=last:
            samples.append(dict(t=round(t,3),reason=key,speed=row['speed_mps'],
                target=target,band=entry.get('rejoin_band')))
        last=key
    return dict(calls=result['calls'],changed_motion_calls=len(changed),reasons=dict(counts),samples=samples)

if __name__=='__main__':
    paths=dict(constant=HERE.parent/'moving_pass_v1/closed_loop_side_pass_2/run1',accelerating=HERE/'baseline_r12/run1')
    out={key:replay(p) for key,p in paths.items()};save(HERE/(sys.argv[1]+'.json'),out)
    for key,v in out.items():print(key,v['calls'],v['changed_motion_calls'],v['reasons'])
    assert out['constant']['changed_motion_calls']==0
    assert any('moving_pass_return_behind' in k for k in out['accelerating']['reasons'])
