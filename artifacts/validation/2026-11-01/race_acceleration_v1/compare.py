"""첫 녹색 표본부터 첫 랩선을 실제로 통과할 때까지 차체 속도를 비교한다."""
import argparse
import json
import numpy as np
from pathlib import Path

BASE=Path(__file__).resolve().parent
OLD=BASE.parent/'corner_approach_v1/solo_10kmh'


def analyze(folder):
    r=json.loads((folder/'report.json').read_text())
    length=r['lap_length_m']
    with (folder/'trajectory.jsonl').open() as f:
        rows=[x for x in map(json.loads,f) if x['light']=='green']
    if len(rows)<2:return dict(passed=r['passed'],complete=False,error=r.get('error'))
    t=np.array([x['time'] for x in rows]);progress=np.array([x['progress_m'] for x in rows])
    v=np.array([np.hypot(x['dynamics']['truth_longitudinal_mps'],x['dynamics']['truth_lateral_mps']) for x in rows])
    if np.any(np.diff(t)<=0) or np.max(np.diff(t))>.25:raise ValueError('연속 차체 속도 표본 부족')
    hits=np.flatnonzero((progress[:-1]<length)&(progress[1:]>=length))
    complete=bool(len(hits))
    if complete:
        i=int(hits[0]);f=(length-progress[i])/(progress[i+1]-progress[i])
        v=np.r_[v[:i+1],v[i]+f*(v[i+1]-v[i])];t=np.r_[t[:i+1],t[i]+f*(t[i+1]-t[i])]
    def time_over(threshold):
        low=np.minimum(v[:-1],v[1:]);hi=np.maximum(v[:-1],v[1:])
        ratio=np.divide(hi-threshold,hi-low,out=(hi>=threshold).astype(float),where=hi-low>1e-10)
        return float(np.dot(np.diff(t),np.clip(ratio,0.,1.)))
    ends=t[t>=t[0]+.30]
    return dict(passed=r['passed'],complete=complete,observed_s=t[-1]-t[0],
        lap_s=t[-1]-t[0] if complete else None,peak_body_mps=float(max(v)),
        above_2_mps_s=time_over(2.),above_95pct_goal_s=time_over(.95*10/3.6),
        max_300ms_drop_mps=float(max(np.interp(ends-.3,t,v)-np.interp(ends,t,v))) if len(ends) else None,
        scope='차체 실제 전·측방 속력의 동일 랩 구간 정의. 속도 설정은 각 실행 보고서에 따르며 미완주는 움직인 창만 표시.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--baseline-only',action='store_true')
    a=p.parse_args();data=dict(baseline=analyze(OLD))
    if not a.baseline_only:data['trial']=analyze(BASE/'run1')
    with (BASE/('reference.json' if a.baseline_only else 'comparison.json')).open('x',encoding='utf-8') as f:
        json.dump(data,f,ensure_ascii=False,indent=2)
