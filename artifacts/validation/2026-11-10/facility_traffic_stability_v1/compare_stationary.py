"""C5와 후속 실행의 신호 통과 뒤 같은 공간 구간 속도를 비교한다. 새 통과 기준을 만들지 않는다."""
import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def analyze(run, continuity_path):
    report=json.loads((run/'report.json').read_text())
    track=ET.parse(run/'runtime_world.sdf').find("./world/model[@name='it_arena_track_static']")
    limits=[]
    for link in track.findall('link'):
        if not link.get('name','').startswith('tl_foot_'): continue
        collision=link.find('collision');pose=list(map(float,collision.findtext('pose').split()))
        dx,dy,_=map(float,collision.findtext('geometry/box/size').split())
        limits.append(pose[1]+.5*dx*abs(math.sin(pose[5]))+.5*dy*abs(math.cos(pose[5])))
    assert len(limits)==2
    limit=max(limits)
    trace=json.loads((run/'trace.json').read_text())
    green=report['case_window']['green_sim_s'];end=report['case_window']['end_sim_s']
    selected=[r for r in trace if green<=r['t']<=end
        and r['y']-.1*abs(math.sin(r['yaw']))-.075*abs(math.cos(r['yaw']))>limit
        and r['y']<=limit+1.2]
    assert selected, 'no post-signal segment'
    times=[r['t'] for r in selected]
    reasons=Counter();zeros=0;commands=[]
    path=run/'follow_decisions.jsonl';digest=hashlib.sha256(path.read_bytes()).hexdigest()
    for line in path.open():
        event=json.loads(line)
        if event['kind']!='call':continue
        data=event['data']; t=data['input']['reference_s']
        if not times[0]<=t<=times[-1]:continue
        v,a,d=data['output']; n=d.get('normal') or d
        reasons[(n.get('reason'),n.get('selector_reason'))]+=1
        zeros+=v<=1e-6;commands.append([t,v])
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
    continuity=json.loads(continuity_path.read_text())
    drops=[r for r in continuity['deceleration_windows']
           if r['start_s']>=times[0] and r['end_s']<=times[-1]]
    low_time=sum(b['t']-a['t'] for a,b in zip(selected,selected[1:]) if a['ground_speed_mps']<.05)
    return dict(run=str(run), sample_time_s=[times[0],times[-1]], signal_foot_front_y_m=limit,
        segment_end_center_y_m=limit+1.2, trace_samples=len(selected),
        min_speed_mps=min(r['ground_speed_mps'] for r in selected),
        max_speed_mps=max(r['ground_speed_mps'] for r in selected),
        sampled_below_005_time_s=low_time, zero_commands=zeros,
        max_sample_time_gap_s=max(b-a for a,b in zip(times,times[1:])),
        reasons=[dict(reason=k[0],selector=k[1],count=v) for k,v in reasons.items()],
        deceleration_windows=drops,input_sha256=digest)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('existing output refused')
    # 참조 continuity는 해당 원시 C5를 분석한 파일이며 원기록 자체는 바꾸지 않는다.
    original=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1/c_stationary_posts/run5'
    result=dict(scope='same physical post-signal straight segment; descriptive comparison, not a new acceptance gate',
                baseline=analyze(original,HERE/'c5_continuity_reference.json'),
                current=analyze(args.run,args.run.parent/'continuity.json'))
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result['current'].items() if k not in ('reasons','deceleration_windows')}))
