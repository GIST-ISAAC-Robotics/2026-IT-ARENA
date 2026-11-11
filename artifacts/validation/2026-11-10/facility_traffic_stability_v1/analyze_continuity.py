"""주행 후 속도 손실 구간과 당시 판단 이유를 함께 추출한다. 통과 기준 변경 없음."""
import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def reason(detail):
    normal=detail.get('normal') or {}
    guide=detail.get('guide') or detail.get('bypass') or {}
    moving=detail.get('moving_pass') or {}
    return dict(behavior=detail.get('behavior'),mode=detail.get('handoff_state'),
                reason=detail.get('reason'),normal_reason=normal.get('reason'),
                normal_behavior=normal.get('behavior'),guide_reason=guide.get('reason'),
                moving_reason=moving.get('reason'),moving_phase=moving.get('phase'))


def analyze(run):
    paths=[run/'report.json',run/'trace.json',run/'follow_decisions.jsonl']
    hashes={p.name:digest(p) for p in paths}
    report=json.loads(paths[0].read_text(encoding='utf8'))
    window=report['case_window'];green=window['green_sim_s'];end=window['end_sim_s']
    rows=[r for r in json.loads(paths[1].read_text(encoding='utf8')) if green<=r['t']<=end]
    assert len(rows)>1 and all(b['t']>a['t'] for a,b in zip(rows,rows[1:])), 'trace time ordering'
    start=next((r['t'] for r in rows if r['ground_speed_mps']>.1),None)
    decisions=[]
    with paths[2].open(encoding='utf8') as stream:
        for line in stream:
            e=json.loads(line)
            if e.get('kind')!='call': continue
            inp=e['data']['input'];speed,steer,detail=e['data']['output']
            t=inp['reference_s']
            if green<=t<=end:
                decisions.append(dict(t=t,command=speed,steer=steer,**reason(detail)))
    stamps=[d['t'] for d in decisions]
    assert all(b>a for a,b in zip(stamps,stamps[1:])), 'decision time ordering'
    def context(a,b):
        selected=decisions[bisect.bisect_left(stamps,a):bisect.bisect_right(stamps,b)]
        counts=Counter((d['mode'],d['behavior'],d['reason'],d['normal_reason'],d['guide_reason']) for d in selected)
        return dict(decisions=len(selected),zero_commands=sum(d['command']<=.001 for d in selected),
            reasons=[dict(mode=k[0],behavior=k[1],reason=k[2],normal_reason=k[3],guide_reason=k[4],count=v)
                     for k,v in counts.most_common(6)],
            min_command=min((d['command'] for d in selected),default=None))
    def describe(a,b):
        selected=rows[a:b+1];first,last=selected[0],selected[-1]
        finite=[r['sampled_gap_m'] for r in selected if isinstance(r.get('sampled_gap_m'),(int,float)) and math.isfinite(r['sampled_gap_m'])]
        return dict(start_s=first['t'],end_s=last['t'],duration_s=last['t']-first['t'],
            start_speed_mps=first['ground_speed_mps'],end_speed_mps=last['ground_speed_mps'],
            min_speed_mps=min(r['ground_speed_mps'] for r in selected),
            start_progress_m=first.get('progress_m'),end_progress_m=last.get('progress_m'),
            start_pose=[first['x'],first['y'],first['yaw']],end_pose=[last['x'],last['y'],last['yaw']],
            min_primary_actor_gap_m=min(finite,default=None),
            safety_reasons=dict(Counter(r.get('safety_reason') for r in selected)),
            **context(first['t']-.10,last['t']+.02))
    lows=[];low_begin=None
    for i,r in enumerate(rows):
        low=start is not None and r['t']>=start and r['ground_speed_mps']<.05
        if low and low_begin is None: low_begin=i
        elif not low and low_begin is not None:
            lows.append(describe(low_begin,i));low_begin=None
    if low_begin is not None:lows.append(describe(low_begin,len(rows)-1))
    times=[r['t'] for r in rows];drops=[]
    for i,r in enumerate(rows):
        if start is None or r['t']<start:continue
        j=bisect.bisect_left(times,r['t']+.3)
        if j>=len(rows):continue
        dt=rows[j]['t']-r['t']
        if dt>.45:continue
        loss=r['ground_speed_mps']-rows[j]['ground_speed_mps']
        if loss>=.3:drops.append((loss/dt,i,j,loss))
    chosen=[]
    for rate,i,j,loss in sorted(drops,reverse=True):
        if any(not (rows[j]['t']<r['start_s'] or rows[i]['t']>r['end_s']) for r in chosen):continue
        row=describe(i,j);row.update(speed_loss_mps=loss,average_deceleration_mps2=rate);chosen.append(row)
        if len(chosen)>=12:break
    commands=[]
    for a,b in zip(decisions,decisions[1:]):
        if start is None or a['t']<start:continue
        dt=b['t']-a['t'];loss=a['command']-b['command']
        if 0<dt<=.12 and loss>=.25:
            commands.append(dict(t=b['t'],dt_s=dt,loss_mps=loss,previous=a,current=b))
    result=dict(scope='post-hoc diagnostics only; sampled 0.3s speed drops are not automatic safety failures',
        run=run.as_posix(),green_s=green,first_motion_s=start,end_s=end,trace_samples=len(rows),decision_samples=len(decisions),
        maximum_trace_gap_s=max(b-a for a,b in zip(times,times[1:])),
        progress_m=rows[-1].get('progress_m'),
        peak_speed_mps=max(r['ground_speed_mps'] for r in rows),
        low_speed_intervals=sorted(lows,key=lambda r:r['duration_s'],reverse=True),
        deceleration_windows=chosen,abrupt_command_drops=commands,
        input_sha256=hashes,input_unchanged=all(digest(p)==hashes[p.name] for p in paths))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('refused: output exists')
    result=analyze(args.run)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(progress_m=result['progress_m'],low_intervals=len(result['low_speed_intervals']),
        longest_low_s=max((r['duration_s'] for r in result['low_speed_intervals']),default=0),
        abrupt_command_drops=len(result['abrupt_command_drops']),
        max_window_decel=max((r['average_deceleration_mps2'] for r in result['deceleration_windows']),default=0),
        input_unchanged=result['input_unchanged']),ensure_ascii=False))
