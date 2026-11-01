"""같은 각본의 실제 moving/closing 결과를 비교한다. 서로 다른 궤적임을 유지한다."""
import argparse
import json
from pathlib import Path
import numpy as np


def summary(folder):
    r=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as f:
        rows=[x for x in map(json.loads,f) if x.get('elapsed_since_green_s') is not None]
    t=np.array([x['elapsed_since_green_s'] for x in rows])
    v=np.array([x['ground_speed_mps'] for x in rows])
    gap=np.array([x['sampled_gap_m'] if x['sampled_gap_m'] is not None else np.nan for x in rows])
    if len(t)<2 or np.any(np.diff(t)<=0) or np.max(np.diff(t))>.25:
        raise ValueError('행동 비교에 필요한 속도 시계열 누락')
    duration=r['case_window']['duration_s']
    stop_case=r['case']=='stop_restart'
    # 0.30초 창의 실제 속도 차이. 순간 슬립이나 물리 엔진의 가속도 피크를 뜻하지 않는다.
    stamps=t[(t>=max(.3,t[0]+.3))&(t<=duration)]
    dv=np.interp(stamps,t,v)-np.interp(stamps-.30,t,v)
    end=float(stamps[np.argmin(dv)])
    seen=[x for x in rows if end-.30<=x['elapsed_since_green_s']<=end]
    first=np.flatnonzero((t>=0)&(t<=duration)&(v>=.05))
    quiet=0.
    if len(first):
        for k in range(first[0],len(t)-1):
            if v[k]<.05:quiet+=max(0.,min(t[k+1],duration)-t[k])
    checkpoints=([3.9,4.5,5.0,8.9,9.5,14.9] if stop_case else [2.,4.,8.,11.9])
    return dict(run=folder.name,case=r['case'],source_unchanged=r['source_unchanged'],
                valid=r['run_valid'],passed=r['passed'],original_failed_gates=[k for k,v in r['gates'].items() if not v],
                progress_m=r['evaluation']['progress']['at_window_end_m'],
                final_gap_m=r['evaluation']['progress']['sampled_gap_at_window_end_m'],
                minimum_gap_m=r['evaluation']['gap']['min_sampled_gap_m'],
                relative_times={str(s):dict(gap_m=float(np.interp(s,t,gap)),ground_mps=float(np.interp(s,t,v)))
                              for s in checkpoints if t[0]<=s<=t[-1]},
                after_first_motion_below_005_s=quiet,max_300ms_speed_drop_mps=max(0.,-float(np.min(dv))),
                largest_drop=dict(start_elapsed_s=end-.30,end_elapsed_s=end,
                    start_mps=float(np.interp(end-.30,t,v)),end_mps=float(np.interp(end,t,v)),
                    sampled_safety_reasons=sorted({str(x['safety_reason']) for x in seen})),
                contact=r['evaluation']['gap']['actor_contact_messages_with_ego'],stop=r['stop_checks'],
                source_hash=r['follow_decisions']['sha256'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('old',type=Path);p.add_argument('new',type=Path)
    p.add_argument('out',type=Path);a=p.parse_args();old=summary(a.old);new=summary(a.new)
    assert old['case']==new['case']
    with a.out.open('x',encoding='utf-8') as f:
        json.dump(dict(moving=old,closing=new),f,indent=2,ensure_ascii=False)
