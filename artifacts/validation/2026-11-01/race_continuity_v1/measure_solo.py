"""공식 코스 단독 주행의 평가용 차체 자세/속도만 읽는다. 출력은 제어에 들어가지 않는다."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np


def measure(folder, target):
    r = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    rows = [row for row in map(json.loads,(folder/'trajectory.jsonl').read_text(encoding='utf-8').splitlines())
            if row['light']=='green']
    if not rows: return {'samples':0,'error':r.get('error','no green trajectory')}
    t=np.array([p['time'] for p in rows])
    v=np.array([math.hypot(p['dynamics']['truth_longitudinal_mps'],p['dynamics']['truth_lateral_mps']) for p in rows])
    dt=np.diff(t)
    if not np.all(dt>0): raise ValueError('non-increasing trajectory timestamps')
    integral=np.r_[0.,np.cumsum((v[:-1]+v[1:])*dt/2)]
    def distance(stamp):
        i=min(len(dt)-1,max(0,int(np.searchsorted(t,stamp,side='right')-1)))
        return integral[i]+(stamp-t[i])*(v[i]+np.interp(stamp,t,v))/2
    sustained=[distance(end)-distance(end-1.) for end in t if end-1.>=t[0]]
    reached_time=reached_m=stopped_time=0.
    threshold=.95*target
    for duration,a,b in zip(dt,v[:-1],v[1:]):
        if max(a,b) <= .05: stopped_time+=duration
        elif min(a,b) < .05: stopped_time+=duration*(.05-min(a,b))/abs(b-a)
        if min(a,b)>=threshold: frac,lo=1.,min(a,b)
        elif max(a,b)<=threshold: continue
        else: frac,lo=(max(a,b)-threshold)/abs(b-a),threshold
        reached_time+=duration*frac
        reached_m+=duration*frac*(lo+max(a,b))/2
    cleanup=r.get('failure_cleanup_stop') or {}
    lap=r.get('lap_length_m', math.inf)
    crossing=None
    for prev,row in zip(rows,rows[1:]):
        a,b=prev['progress_m'],row['progress_m']
        if a<lap<=b:
            crossing=prev['time']+(row['time']-prev['time'])*(lap-a)/(b-a)
            break
    return dict(profile=r.get('speed_profile'),passed=r.get('passed'),error=(r.get('error') or '').split(':',1)[0] or None,target_mps=target,
                progress_m=r.get('progress_m',rows[-1]['progress_m']),completed_laps=r.get('completed_laps'),
                collision_samples=r.get('collision_samples'),stop_stable=r.get('stop_stable'),samples=len(rows),
                failure_cleanup_stop_ack=cleanup.get('acknowledged'),
                failure_cleanup_dense_stop=cleanup.get('dense_pose_stop_passed'),
                first_green_sample_s=float(t[0]),last_green_sample_s=float(t[-1]),span_s=float(t[-1]-t[0]),
                lap_s_from_first_green_sample=(None if crossing is None else float(crossing-t[0])),
                maximum_sample_gap_s=float(max(dt)) if len(dt) else None,peak_ground_mps=float(v.max()),
                best_one_second_mean_mps=max(sustained,default=None),
                time_above_95_percent_s=reached_time,distance_above_95_percent_m=reached_m,
                time_ground_at_most_0p05_mps_s=stopped_time,
                planner_published_peak_mps=max((row['autonomy']['speed_command_mps'] for row in rows
                    if row['autonomy'].get('speed_command_mps') is not None),default=None),
                safe_command_peak_mps=max((row['safe_drive_speed_mps'] for row in rows
                    if row.get('safe_drive_speed_mps') is not None),default=None),
                marker_ids=r.get('marker_ids'),shutdown_clean=r.get('shutdown_clean'),
                safety_reasons=dict(Counter((p.get('safety') or {}).get('reason') for p in rows)),
                scope='sampled green window; forward/lateral ground speed, linear interpolation; not wheel speed or physical certification')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('folder',type=Path)
    parser.add_argument('target_mps',type=float)
    args=parser.parse_args()
    print(json.dumps(measure(args.folder,args.target_mps),indent=2))
