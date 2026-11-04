"""첫 주행 허가~첫 결승 통과(미완주면 STOP 전) 속도 전달 대조. 평가 전용."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


def load(p):
    return json.loads(p.read_bytes())


def summary(series, begin, end, *, linear=False):
    if not series:
        return None
    assert all(math.isfinite(t) and math.isfinite(v) for t,v in series)
    assert all(b[0]>=a[0] for a,b in zip(series,series[1:]))
    # 한 수신 시각에 갱신이 여럿이면 마지막 값만 남긴다. 첫/끝을 시계열 밖으로 외삽하지 않는다.
    series=list(dict(series).items())
    ts=[s[0] for s in series]
    a=max(begin,ts[0]);b=min(end,ts[-1])
    if b<=a:return None
    def at(t):
        i=bisect_right(ts,t)-1; x,y=series[i]
        if not linear or t==x or i+1==len(series):return y
        u,v=series[i+1]
        return y+(v-y)*(t-x)/(u-x)
    samples=[(a,at(a))]+[(t,v) for t,v in series if a<t<b]+[(b,at(b))]
    def over(threshold):
        total=0.
        for (t,v),(u,w) in zip(samples,samples[1:]):
            if not linear or v==w:total+=(u-t)*(v>=threshold)
            elif min(v,w)>=threshold:total+=u-t
            elif max(v,w)>threshold:total+=(u-t)*(max(v,w)-threshold)/abs(w-v)
        return total
    when,peak=max(samples[:-1] if not linear else samples,key=lambda k:k[1])
    return dict(peak_mps=peak,peak_t_s=when,above_2p5_s=over(2.5),above_95pct_s=over(.95*10/3.6),
                above_10kmh_s=over(10/3.6),coverage_s=b-a,max_sample_gap_s=max(
                    (u[0]-t[0] for t,u in zip(samples,samples[1:])),default=None))


def measure(run):
    rep=load(run/'report.json')
    trace=load(run/'trace.json')
    mcu=load(run/'actuation_trace.json')
    start=rep['start_time_sim_s']; finish=None
    if start is None or not trace or not mcu:return dict(measured=False,reason='no start or observations')
    for a,b in zip(trace,trace[1:]):
        lap=rep['lap_length_m']
        if b['time']>=start and a['progress_m']<lap<=b['progress_m']:
            finish=a['time']+(b['time']-a['time'])*(lap-a['progress_m'])/(b['progress_m']-a['progress_m'])
            break
    stop=next((r['now_us']*1e-6 for r in mcu if r['reason']=='software_stop' and r['now_us']*1e-6>=start),None)
    end=min(t for t in (finish,stop,trace[-1]['time']) if t is not None)
    def series(rows,t,v):
        out=[]
        for row in rows:
            x=t(row);y=v(row)
            if x is not None and y is not None:out.append((float(x),float(y)))
        return out
    dyn=lambda r:r.get('dynamics') or {}
    auto=lambda r:r.get('autonomy') or {}
    dyn_t=lambda r:dyn(r).get('sim_time_s')
    mcu_t=lambda r:r['now_us']*1e-6
    paths={
        'planner_status':(series(trace,lambda r:auto(r).get('sim_time_s'),lambda r:auto(r).get('speed_command_mps')),False),
        'safe_command_sample':(series(trace,lambda r:r['time'],lambda r:r.get('safe_drive_speed_mps')),False),
        'mcu_target':(series(mcu,mcu_t,lambda r:r['target_speed_mps']),False),
        'actuator_applied_target':(series(trace,dyn_t,lambda r:dyn(r).get('applied_target_mps')),False),
        'motor_encoder_equivalent':(series([r for r in mcu if r.get('feedback_valid')],lambda r:r['feedback_capture_us']*1e-6,
                                    lambda r:r['mean_wheel_mps']),False),
        'ground':(series(trace,dyn_t,lambda r:math.hypot(dyn(r)['truth_longitudinal_mps'],dyn(r)['truth_lateral_mps'])
                        if 'truth_longitudinal_mps' in dyn(r) else None),True)}
    return dict(measured=True,start_s=start,first_finish_s=finish,excluded_explicit_stop_s=stop,end_s=end,
        values={k:summary(v,start,end,linear=linear) for k,(v,linear) in paths.items()},
        scope='계획 상태·보호 출력은 요약 표본으로 일부 갱신이 생략된다. 보호 명령은 수신 표본 시각이며 나머지는 기록된 원천 시각. 명령·엔코더는 단계 유지, 차체는 평가 속도 선형 보간. 서로 다른 신호의 최고를 같은 순간으로 가정하지 않는다.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args()
    with args.output.open('x',encoding='utf8') as f:
        json.dump(measure(args.run),f,indent=2,ensure_ascii=False,allow_nan=False)
