"""코스 대기를 제어/보호 정지와 양수 명령 중 정체로 나누는 평가-only 요약."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


def finite(value):return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def kind(row):
    if not finite(row.get('ground_speed_mps')) or row['ground_speed_mps']>=.02:return None
    drive,safe=row.get('drive_speed_mps'),row.get('safe_speed_mps')
    if not finite(drive) or not finite(safe):return 'missing_command'
    if drive<=1e-9:return 'controller_zero'
    if safe<=1e-9:return 'protection_zero'
    return 'positive_command_low_body_speed'


def stats(rows,key,diagnostic=False):
    values=[(r.get('braking_diagnostics') or {}).get(key) if diagnostic else r.get(key) for r in rows]
    values=[v for v in values if finite(v)]
    return dict(samples=len(values),minimum=min(values),median=statistics.median(values),maximum=max(values)) if values else None


def summarize(group):
    spins=[]
    for row in group:
        d=row.get('braking_diagnostics') or {};a=d.get('left_speed_rad_s');b=d.get('right_speed_rad_s')
        if finite(a) and finite(b) and max(abs(a),abs(b))>3 and min(abs(a),abs(b))<1:spins.append(row)
    return dict(start_s=group[0]['t'],end_s=group[-1]['t'],duration_s=group[-1]['t']-group[0]['t'],
        start_progress_m=group[0].get('progress_m'),end_progress_m=group[-1].get('progress_m'),
        samples=len(group),categories=dict(Counter(kind(r) for r in group)),
        safety_reasons=dict(Counter(r.get('safety_reason') for r in group)),
        positive_safe_samples=sum((r.get('safe_speed_mps') or 0)>.1 for r in group),
        asymmetric_wheel_samples=len(spins),body_speed=stats(group,'ground_speed_mps'),safe_speed=stats(group,'safe_speed_mps'),
        left_wheel=stats(group,'left_speed_rad_s',True),right_wheel=stats(group,'right_speed_rad_s',True),
        wheel_mean=stats(group,'wheel_surface_speed_mps',True),torque=stats(group,'motor_torque_nm',True))


def analyze(run):
    path=run/'trajectory.jsonl';before=hashlib.sha256(path.read_bytes()).hexdigest()
    report=json.loads((run/'report.json').read_text(encoding='utf-8'));start=report.get('green_observed_sim_s');end=(report.get('case_window') or {}).get('end_sim_s')
    if start is None or end is None:raise ValueError('missing drive window')
    with path.open(encoding='utf-8') as stream:rows=[json.loads(line) for line in stream]
    rows=[r for r in rows if start<=r['t']<=end]
    moving=next((r['t'] for r in rows if finite(r.get('ground_speed_mps')) and r['ground_speed_mps']>=.1),None)
    groups=[];current=[]
    for row in rows:
        if moving is None or row['t']<moving:continue
        if current and row['t']-current[-1]['t']>.25:groups.append(current);current=[]
        if kind(row) is not None:current.append(row)
        elif current:groups.append(current);current=[]
    if current:groups.append(current)
    windows=[summarize(g) for g in groups if g[-1]['t']-g[0]['t']>=.5]
    return dict(run=str(run),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_sha256=before,input_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==before,
        scope='evaluation-only low-speed windows: body<.02 m/s for>=.5s after first>=.1m/s; not stall cause or real slip proof',
        green_s=start,end_s=end,first_motion_s=moving,windows=windows,
        maximum_low_speed_window_s=max((w['duration_s'] for w in windows),default=0),
        wheel_spin_note='>3 rad/s on one rear wheel and <1 on the other is a diagnostic flag only; contact and no-load require further evidence')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=analyze(args.run)
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('windows','scope','wheel_spin_note')}))
