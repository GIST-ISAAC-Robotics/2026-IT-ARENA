"""완주 시간 중 속도대별 비중. 녹색 첫 표본~랩선만 사용한다."""
from collections import defaultdict
import json
import math
from pathlib import Path


def summarize(folder):
    report=json.loads((folder/'report.json').read_text())
    finish=float(report['lap_length_m'])
    with (folder/'trajectory.jsonl').open() as stream:
        samples=[r for r in map(json.loads,stream) if r['light']=='green']
    def speed(p):
        d=p['dynamics']
        return math.hypot(d['truth_longitudinal_mps'],d['truth_lateral_mps'])
    bounds=(0.,.05,.5,1.,1.5,1.9,float('inf'))
    times=[0.]*(len(bounds)-1); commands=defaultdict(float); elapsed=0.; completed=False
    for a,b in zip(samples,samples[1:]):
        step=b['time']-a['time']
        if step<=0 or step>.25:raise ValueError('trajectory not dense enough')
        fraction=1.
        if a['progress_m']<finish<=b['progress_m']:
            fraction=(finish-a['progress_m'])/(b['progress_m']-a['progress_m']);completed=True
        dt=step*fraction;v0=speed(a);v1=v0+fraction*(speed(b)-v0)
        if abs(v1-v0)<1e-12:
            i=next(i for i in range(len(times)) if bounds[i]<=v0<bounds[i+1]); times[i]+=dt
        else:
            low,high=sorted((v0,v1))
            for i in range(len(times)):
                times[i]+=dt*max(0.,min(high,bounds[i+1])-max(low,bounds[i]))/(high-low)
        elapsed+=dt
        # 구간 시작의 수신 상태를 사용한 약 10 Hz 표본이다. 원인별 정확한 구속 시간은 아니다.
        d=a['autonomy'];command=float(a['safe_drive_speed_mps'])
        label=('zero' if command<.001 else 'positive_exceeds_1' if command>1.001 else
               'positive_at_or_below_1')
        commands[label]+=dt
        if completed:break
    if not completed or abs(sum(times)-elapsed)>1e-5:raise ValueError('incomplete lap accounting')
    return dict(lap_s=elapsed,average_course_speed_mps=finish/elapsed,body_seconds=dict(zip(
        ('0-.05','.05-.5','.5-1','1-1.5','1.5-1.9','1.9+'),times)),
        safe_command_seconds_first_sample=dict(commands),scope='실제 차체 속도 선형보간; 경로 평균=공식 랩거리/경과시간')


if __name__=='__main__':
    here=Path(__file__).resolve().parent
    results={p.name:summarize(p) for p in (here.parent/'race_continuity_v1/solo_1p5_race_path',here/'solo_2p0')}
    with (here/'lap_budget.json').open('x',encoding='utf-8') as f:json.dump(results,f,ensure_ascii=False,indent=2)
    print(json.dumps(results,ensure_ascii=False))
