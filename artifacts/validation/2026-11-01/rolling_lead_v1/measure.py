"""각본 속도 대신 저장된 자차·상대 자세와 센서 판단으로 같은 3개 구간을 평가한다."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

import numpy as np

from course import BASE,SCHEDULE


def interval_below(t,v,threshold):
    # 인접한 두 속도 표본 사이를 선형 보간하여 임계값 아래인 시간만 센다.
    lo=np.minimum(v[:-1],v[1:]);hi=np.maximum(v[:-1],v[1:])
    frac=np.divide(threshold-lo,hi-lo,out=(v[:-1]<threshold).astype(float),where=hi-lo>1e-10)
    return float(np.dot(np.diff(t),np.clip(frac,0.,1.)))


def read(run):
    r=json.loads((run/'report.json').read_bytes())
    ev=r['evaluation'];g=r['case_window']['green_sim_s']
    with (run/'trajectory.jsonl').open(encoding='utf-8') as f:
        rows=[x for x in map(json.loads,f) if x.get('sampled_gap_m') is not None and g-.20<=x['t']<=g+17.20]
    vals=np.asarray([[x['t']-g,x['ground_speed_mps'],x['progress_m'],x['sampled_gap_m']] for x in rows],float)
    if len(vals)<2 or not np.all(np.isfinite(vals)) or np.min(np.diff(vals[:,0]))<=0 or np.max(np.diff(vals[:,0]))>.20:
        raise ValueError('속도/진행/간격의 시간 범위 불충분')
    with (run/'actor_poses.jsonl').open(encoding='utf-8') as f:
        actor=np.asarray([p for p in map(json.loads,f) if g-.05<=p[0]<=g+17.05],float)
    if len(actor)<2 or not np.all(np.isfinite(actor)) or np.min(np.diff(actor[:,0]))<=0 or np.max(np.diff(actor[:,0]))>.05:
        raise ValueError('실제 상대 자세의 시간 범위 불충분')
    at=actor[:,0]-g;travel=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(actor[:,1:3],axis=0),axis=1))]
    reasons=[Counter() for _ in SCHEDULE];states=[Counter() for _ in SCHEDULE];patch=[0 for _ in SCHEDULE]
    h=sha256()
    with (run/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            h.update(line);e=json.loads(line)
            if e['kind']!='call':continue
            d=e['data'];s=d['input']['reference_s']-g;q=d['output'][2]
            for i,(a,b,_) in enumerate(SCHEDULE):
                if a<=s<b:
                    st=q.get('handoff_state');states[i][st]+=1
                    info=q.get('follow_entry_space',{})
                    if info.get('rear_axle_to_patch_m') is not None:patch[i]+=1
                    reasons[i][info.get('reason','other')]+=1
    if h.hexdigest()!=r['follow_decisions']['sha256']:raise ValueError('판단 기록 변경')
    t=vals[:,0];phases=[]
    for i,(a,b,v) in enumerate(SCHEDULE):
        lo=max(a,t[0],at[0]);hi=min(b,t[-1],at[-1])
        if hi<=lo or lo-a>.20 or b-hi>.20:raise ValueError('각본 구간 대표성 부족')
        stamps=np.r_[lo,t[(lo<t)&(t<hi)],hi]
        speed,prog,gap=[np.interp(stamps,t,vals[:,k]) for k in (1,2,3)]
        actual=np.interp([lo,hi],at,travel)
        phases.append(dict(registered_window_s=[a,b],covered_s=[lo,hi],lead_script_mps=v,
            actual_lead_path_m=float(actual[1]-actual[0]),ego_course_progress_m=prog[-1]-prog[0],
            ego_mean_body_mps=float(np.trapz(speed,stamps)/(hi-lo)),ego_peak_body_mps=max(speed),
            body_below_005_s=interval_below(stamps,speed,.05),gap_start_m=gap[0],gap_end_m=gap[-1],
            observed_patch_calls=patch[i],controller_states=dict(states[i]),entry_reasons=dict(reasons[i])))
    return dict(run_valid=r['run_valid'],raw_pass=r['passed'],raw_fail=[k for k,v in r['gates'].items() if not v],
        original_gates=r['gates'],stop=r['stop_checks'],signal=r.get('team_signal_evidence'),curve=ev['case_metrics'],
        min_swept_gap_m=ev['gap']['min_swept_lower_bound_m'],contacts=ev['gap']['actor_contact_messages_with_ego'],
        phases=phases,scope='상대·자차 자세는 사후 평가만 사용. 전방 반환 조각이 실제 특정 차라는 신원 판정이 아니다.')


if __name__=='__main__':
    with (BASE/'summary.json').open('x',encoding='utf-8') as f:json.dump(read(BASE/'run1'),f,ensure_ascii=False,indent=2)
