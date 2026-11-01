"""녹색 뒤 속도 판단을 실제로 제한한 항목과 보호 상태 표본을 구분한다(사후 분석)."""
from collections import Counter
import argparse
from hashlib import sha256
import json
from pathlib import Path

import numpy as np


def binding(d):
    v,_,q=d['output']
    state=q.get('handoff_state','unavailable')
    if state!='FOLLOW':
        return state
    if (q.get('relative_yield') or {}).get('final_command_clipped'):
        return 'yield'
    if (q.get('entry_space') or {}).get('hold') and v==0:
        return 'prepare_hold'
    n=q.get('normal') or {}
    if n.get('behavior')=='HOLD':
        return 'G1:'+str(n.get('reason'))
    f=q.get('follow_entry_space') or {}
    if f.get('applied') and f.get('before_mps',v)>v+1e-7:
        return 'entry:'+str(f.get('reason'))
    if (q.get('preparation') or {}).get('applied'):
        return 'prepare_cap'
    if v==0:
        return 'other_zero:'+str(q.get('reason'))
    return 'G1_or_unchanged'


def read(run):
    r=json.loads((run/'report.json').read_bytes());win=r['case_window']
    green=win['green_sim_s'];duration=win['duration_s'];sha=sha256()
    if r['case']=='first_curve' and duration==17.:
        cuts=(0.,6.,13.,17.)
    else:
        cuts=(0.,min(4.,duration),duration)
    by_phase=[Counter() for _ in cuts[:-1]];zeros=[Counter() for _ in by_phase]
    calls=[];last_seq=-1;footer=None
    with (run/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            sha.update(line);e=json.loads(line)
            if e['seq']!=last_seq+1:raise ValueError('판단 순번 누락')
            last_seq=e['seq'];d=e['data']
            if e['kind']=='footer':footer=d
            if e['kind']=='call':
                calls.append((d['input']['reference_s']-green,d['output'][0],binding(d)))
    if (sha.hexdigest()!=r['follow_decisions']['sha256'] or not footer or footer['written']!=last_seq
            or any(footer[k] for k in ('failed','dropped','capture_errors','pending'))):
        raise ValueError('판단 기록 불일치')
    # 다음 호출까지 실제 지속한 시간만 누적한다. 호출이 끊기면 미분류로 둔다.
    for (t,v,k),(u,_,_) in zip(calls,calls[1:]):
        if u<t:raise ValueError('서로 다른 시간축')
        if u-t>.05:continue
        for j,(a,b) in enumerate(zip(cuts,cuts[1:])):
            dt=min(b,u)-max(a,t)
            if dt>0:
                by_phase[j][k]+=dt
                if v<=.001:zeros[j][k]+=dt
    with (run/'trajectory.jsonl').open(encoding='utf-8') as f:
        rows=[x for x in map(json.loads,f) if x.get('elapsed_since_green_s') is not None]
    t=np.array([x['t']-green for x in rows]);speeds=np.array([x['ground_speed_mps'] for x in rows])
    if len(t)<2 or np.min(np.diff(t))<=0 or np.max(np.diff(t))>.25 or not np.isfinite(speeds).all():
        raise ValueError('차체 표본 결측')
    result=[]
    for j,(a,b) in enumerate(zip(cuts,cuts[1:])):
        # 구간 안의 끝점마다 같은 0.30초 폭을 비교한다.
        grid=t[(t>=max(t[0],a)+.3)&(t<=b)]
        if not len(grid):raise ValueError('구간 대표성 부족')
        drops=np.interp(grid-.3,t,speeds)-np.interp(grid,t,speeds)
        i=int(np.argmax(drops));end=float(grid[i]);start=end-.30
        inside=[x for x in rows if start<=x['t']-green<=end]
        causes=Counter(k for s,_,k in calls if start<=s<=end)
        result.append(dict(window_s=[a,b],binding_s=dict(by_phase[j]),zero_s=dict(zeros[j]),
            unclassified_s=b-a-sum(by_phase[j].values()),worst_drop_030_mps=max(0.,float(drops[i])),
            drop_window_s=[start,end],body_mps=[float(np.interp(x,t,speeds)) for x in (start,end)],
            drop_planner_calls=dict(causes),drop_guard_samples=dict(Counter(str(x['safety_reason']) for x in inside)),
            phase_guard_samples=dict(Counter(str(x['safety_reason']) for x in rows if a<=x['t']-green<b))))
    return dict(trace_sha256=sha.hexdigest(),case=r['case'],phases=result,
                scope='속도 결정의 발행 전 제한과 별도 10 Hz 보호 상태 표본. 같은 원인만으로 차체 감속이 전부 설명된다는 뜻은 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
    report=read(a.run)
    with a.out.open('x',encoding='utf-8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
