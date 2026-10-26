"""앞차 정지 당시 실제 자차 속도와 간격을 원시 자세에서 분해한다(평가 전용)."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import g1_traffic_eval as ev


def analyze(run):
    files=('report.json','trace.json','ego_poses.jsonl','actor_poses.jsonl','follow_decisions.jsonl')
    hashes={f:hashlib.sha256((run/f).read_bytes()).hexdigest() for f in files}
    report=json.loads((run/'report.json').read_text(encoding='utf8'))
    green=report['evaluation']['t_green']
    trace=json.loads((run/'trace.json').read_text(encoding='utf8'))
    ego=[json.loads(l) for l in (run/'ego_poses.jsonl').read_text(encoding='utf8').splitlines()]
    near=[json.loads(l) for l in (run/'actor_poses.jsonl').read_text(encoding='utf8').splitlines()]
    windows={}
    for name,rows in [('ego',ego),('lead',near)]:
        segments=ev.kinematic_speed_segments(rows)
        windows[name]={}
        for label,first,last in [('before_stop',3.5,4.),('after_stop',4.,5.),('before_restart',8.5,9.),('after_restart',9.,10.)]:
            selected=[s for s in segments if s['t0']>=green+first and s['t1']<=green+last]
            speeds=[s['average_speed_mps'] for s in selected]
            windows[name][label]={'sim_window_s':[green+first,green+last],
                'coverage':ev.pose_window_coverage(segments,green+first,green+last)['complete'],
                'count':len(speeds),'speed_min_mps':min(speeds,default=None),
                'speed_max_mps':max(speeds,default=None),'speed_median_mps':float(np.median(speeds)) if speeds else None}
    t_stop=green+4.
    before=[r for r in trace if r['t']<=t_stop]
    pre=before[-1] if before else None
    first_zero=next((r for r in trace if r['t']>=t_stop and r['safe_speed_mps']==0.),None)
    reasons={}
    threat_calls=0
    for line in (run/'follow_decisions.jsonl').open(encoding='utf8'):
        event=json.loads(line)
        if event['kind']!='call':
            continue
        detail=event['data']['output'][2]
        reasons[detail.get('reason','unknown')]=reasons.get(detail.get('reason','unknown'),0)+1
        threats=(detail.get('m3_lateral') or {}).get('threats') or {}
        threat_calls+=int(any(v.get('present') for v in threats.values()))
    return {'run':str(run),'original_passed':report['passed'],'run_valid':report['run_valid'],
        'speed_windows':windows,'last_pre_stop_trace':({k:pre.get(k) for k in
            ('t','ground_speed_mps','safe_speed_mps','drive_speed_mps','sampled_gap_m')} if pre else None),
        'first_zero_safe_after_stop_delay_s':first_zero['t']-t_stop if first_zero else None,
        'm3_threat_calls':threat_calls,'decision_reason_counts':reasons,'input_sha256':hashes,
        'inputs_unchanged':all(hashlib.sha256((run/f).read_bytes()).hexdigest()==v for f,v in hashes.items()),
        'scope':'post-hoc finite pose/trace diagnostics, no verdict or control change; not measured hardware braking'}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    assert not args.output.exists()
    result=analyze(args.run)
    assert result['inputs_unchanged']
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False))
