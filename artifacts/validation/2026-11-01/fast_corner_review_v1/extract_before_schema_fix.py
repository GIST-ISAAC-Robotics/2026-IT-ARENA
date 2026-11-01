"""기존 빠른 코너 실패의 기록 시간축을 대조한다. 새 제어 재생이나 주행은 아니다."""
from bisect import bisect_left
from collections import Counter
from hashlib import sha256
import csv
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def extract(name):
    run=HERE.parent/name/'run1'
    trace=json.loads((run/'trace.json').read_bytes())
    positions=[row for row in trace if 16.5<=row.get('progress_m',-1)<=20.5]
    if not positions:raise ValueError('corner positions missing')
    first,last=positions[0]['t'],positions[-1]['t']
    decisions=[];digest=sha256()
    with (run/'follow_decisions.jsonl').open('rb') as stream:
        for line in stream:
            digest.update(line);event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];stamp=data['input']['reference_s']
            if first-.1<=stamp<=last+.1:decisions.append(data)
    times=[d['input']['reference_s'] for d in decisions]
    if not times:raise ValueError('corner decisions missing')
    rows=[]
    for state in positions:
        index=bisect_left(times,state['t'])
        index=min((i for i in (index-1,index) if 0<=i<len(times)),key=lambda i:abs(times[i]-state['t']))
        data=decisions[index];delta=times[index]-state['t']
        if abs(delta)>.04:continue
        speed,steering,detail=data['output']
        body=state.get('truth_longitudinal_mps',state['ground_speed_mps'])
        measured=data['input']['speed_mps']
        rows.append(dict(t_s=state['t'],decision_time_offset_s=delta,progress_m=state['progress_m'],
            body_truth_mps=body,controller_measured_mps=measured,body_minus_measured_mps=body-measured,
            target_mps=speed,steering_rad=steering,reason=detail.get('reason'),
            safety=state.get('safety_reason'),boundary_age_s=detail.get('boundary_age_s'),
            boundary_from_memory=detail.get('boundary_from_memory'),path_wall=detail.get('path_wall'),
            wall_fallback=detail.get('wall_fallback'),path_ahead_m=detail.get('observed_path_ahead_m'),
            swept_clearance_m=detail.get('swept_clearance_m'),
            corridor_reason=(detail.get('corridor_candidate') or {}).get('reason')))
    if not rows:raise ValueError('time matching failed')
    with (HERE/f'{name}.csv').open('x',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    moving=[r for r in rows if r['body_truth_mps']>.5]
    worst=max(moving,key=lambda r:r['body_minus_measured_mps']) if moving else None
    zero=next((r for r in rows if r['target_mps']==0.),None)
    return dict(run=name,input_sha256={
        'trace.json':sha256((run/'trace.json').read_bytes()).hexdigest(),
        'follow_decisions.jsonl':digest.hexdigest()},matched_rows=len(rows),
        first_zero=zero,largest_body_minus_measured_while_moving=worst,
        reason_counts=dict(Counter(r['reason'] for r in rows)),
        sensor_timing_limit='차체 표본 시각과 제어 판단 reference 시각을 최대 40 ms 이내 대응. 실제 센서 capture 시각과 같다고 가정하지 않음.')


if __name__=='__main__':
    result=dict(scope='같은 코너의 기존 궤적/명령/센서 속도 대조. 정답은 평가 전용이며 속도 차이의 물리 원인을 확정하지 않는다.',
                script_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
                cases=[extract(name) for name in ('solo_route_v1','solo_headway_v1')])
    with (HERE/'summary.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))
