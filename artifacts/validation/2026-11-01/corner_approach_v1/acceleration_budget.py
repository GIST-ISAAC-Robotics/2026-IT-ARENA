"""같은 10 km/h 주행의 계획 명령에 발행 증속 제한만 대조한다. 경로·감속·실차 가속력 변경 없음."""
import json
import math
from pathlib import Path
import numpy as np

BASE=Path(__file__).resolve().parent


def calculate():
    folder=BASE/'solo_10kmh'
    audit=json.loads((BASE/'solo_10kmh_metrics.json').read_text())
    start=audit['first_green_sample_s'];end=start+audit['lap_s_from_first_green_sample']
    rates=(1.,1.5,2.);prev=[0.]*len(rates);out=[];prior=None;large_gaps=0
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for r in map(json.loads,f):
            if r['kind']=='reset':prev=[0.]*len(rates);prior=None;continue
            if r['kind']!='call':continue
            d=r['data'];t=d['input']['reference_s']
            if t>end:break
            if prior is not None and t-prior>.03:
                # 기록되지 않은 타이머 동작은 재구성하지 않는다. 실제 발행과의 일치도를 별도로 확인.
                large_gaps+=1;prev=[0.]*len(rates)
            prior=t;target=d['output'][0]
            prev=[min(target,p+a/50) for a,p in zip(rates,prev)]
            if t>=start:out.append([t,target,*prev])
    arr=np.array(out)
    stamps={round(x[0],6):x[2] for x in arr};errors=[]
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as f:
        for row in map(json.loads,f):
            a=row.get('autonomy') or {};t=a.get('reference_stamp_s');v=a.get('speed_command_mps')
            if t is not None and v is not None and start<=t<=end and round(t,6) in stamps:
                errors.append(abs(v-stamps[round(t,6)]))
    if not errors:raise ValueError('기존 발행과 대조할 공통 시각 없음')
    stats=[]
    dt=np.diff(arr[:,0]);dt=np.where((dt>0)&(dt<.03),dt,0.)
    for j,rate in enumerate(rates):
        vs=arr[:,2+j];series=vs[:-1]
        stats.append(dict(acceleration_mps2=rate,maximum_mps=float(max(vs)),
            command_distance_difference_m=float(np.sum(dt*(series-arr[:-1,2]))),
            commanded_above_2mps_s=float(np.sum(dt[series>=2.])),
            commanded_above_95percent_s=float(np.sum(dt[series>=.95*10/3.6])),
            changed_zero=int(np.sum((vs==0)!=(arr[:,2]==0)))))
    return dict(scope='동일 센서 시계열에서 발행 직전 50 Hz 상향 제한만 계산한 수치. 실제 랩·차체 가속·제동거리 아님.',
        lap_window_s=[start,end],unrecorded_intervals=large_gaps,raw_calls=len(arr),
        baseline_observed_samples=len(errors),baseline_max_difference_mps=max(errors),
        baseline_exact_fraction=float(np.mean(np.array(errors)<1e-7)),raw_peak_mps=float(max(arr[:,1])),
        candidates=stats)


if __name__=='__main__':
    result=calculate()
    with (BASE/'acceleration_budget.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
