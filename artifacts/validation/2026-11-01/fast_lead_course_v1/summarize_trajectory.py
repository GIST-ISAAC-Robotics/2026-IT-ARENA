"""완료된 평가 기록의 속도·랩·출력 제한을 요약한다. 제어기에 쓰지 않는다."""
import json,math,sys
from pathlib import Path
from collections import Counter
from hashlib import sha256


def summarize(run):
    report=json.loads((run/'report.json').read_bytes())
    green=report['case_window']['green_sim_s'];duration=report['case_window']['duration_s']
    rows=[json.loads(line) for line in (run/'trajectory.jsonl').read_text().splitlines()]
    rows=[r for r in rows if green<=r['t']<=green+duration]
    assert rows and all(a['t']<b['t'] for a,b in zip(rows,rows[1:]))
    covered=rows[-1]['t']-rows[0]['t']
    lap=46.6329
    crossing=None
    for a,b in zip(rows,rows[1:]):
        if a['progress_m']<lap<=b['progress_m']:
            crossing=a['t']-green+(b['t']-a['t'])*(lap-a['progress_m'])/(b['progress_m']-a['progress_m'])
            break
    bins=[]
    for lo,hi in [(0.,4.),(4.,9.),(9.,20.),(20.,40.),(40.,60.),(60.,duration)]:
        hi=min(hi,duration)
        selected=[r for r in rows if lo<=r['t']-green<=hi]
        if len(selected)<2:continue
        times=Counter();weight=0.;integral=0.
        for a,b in zip(selected,selected[1:]):
            dt=b['t']-a['t'];weight+=dt
            integral+=dt*(a['ground_speed_mps']+b['ground_speed_mps'])/2
            times[a['safety_reason']]+=dt
        bins.append(dict(from_s=lo,to_s=hi,sampled_span_s=weight,
            progress_m=selected[-1]['progress_m']-selected[0]['progress_m'],
            mean_body_speed_mps=integral/weight,max_body_speed_mps=max(r['ground_speed_mps'] for r in selected),
            final_sample_gap_m=selected[-1]['sampled_gap_m'],safety_reason_seconds=dict(times)))
    return dict(scope='완료된 평가 자세/속도만 사용. 10 Hz 표본 최대는 지속 속도나 실물 성능 보장이 아님.',
        trajectory_sha256=sha256((run/'trajectory.jsonl').read_bytes()).hexdigest(),
        green_sim_s=green,requested_window_s=duration,sampled_window=[rows[0]['t']-green,rows[-1]['t']-green],
        maximum_sample_gap_s=max(b['t']-a['t'] for a,b in zip(rows,rows[1:])),
        lap_length_m=lap,lap_crossing_s=crossing,peak_body_mps=max(r['ground_speed_mps'] for r in rows),
        bins=bins)


if __name__=='__main__':
    value=summarize(Path(sys.argv[1]))
    with Path(sys.argv[2]).open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2)
    print(json.dumps(value,ensure_ascii=False,indent=2))
