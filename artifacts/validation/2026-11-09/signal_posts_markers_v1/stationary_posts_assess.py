"""정지차 원목표에 실제 신호 지주/발판 통과를 더한 별도 평가. 사전 C5용."""
import json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path

def assess(run):
    run=Path(run);read=lambda p:json.loads(p.read_text(encoding='utf8'))
    goal=read(run.parent/f'{run.name}_goal_surface_corrected.json')
    posts=read(run/'dense_post_clearance.json');report=read(run/'report.json')
    track=ET.parse(run/'runtime_world.sdf').find("./world/model[@name='it_arena_track_static']")
    limits=[]
    for link in track.findall('link'):
        if not link.get('name','').startswith('tl_foot_'):continue
        collision=link.find('collision');pose=list(map(float,collision.findtext('pose').split()))
        dx,dy,_=map(float,collision.findtext('geometry/box/size').split())
        limits.append(pose[1]+.5*dx*abs(math.sin(pose[5]))+.5*dy*abs(math.cos(pose[5])))
    assert len(limits)==2
    limit=max(limits);end=report['case_window']['end_sim_s'];crossed=None
    for line in (run/'ego_poses.jsonl').open():
        t,x,y,yaw=json.loads(line)
        if t<=end and y-.10*abs(math.sin(yaw))-.075*abs(math.cos(yaw))>limit:
            crossed=t;break
    gates=dict(goal_passed=goal['passed'],
        signal_posts_feet_clear=posts['samples']>0 and len(posts['parts'])==4 and
            all(x['overlap_samples']==0 for x in posts['parts'].values()),
        entire_body_passed_signal_before_stop=crossed is not None,
        no_process_leftover=not (run.parent/f'postflight_processes_{run.name}.txt').read_text().strip())
    result=dict(passed=all(gates.values()),gates=gates,goal_metrics=goal['metrics'],
        full_body_passage_s=crossed,signal_foot_front_y_m=limit,
        min_post_gap_m=min(x['minimum_m'] for k,x in posts['parts'].items() if 'post' in k),
        min_foot_gap_m=min(x['minimum_m'] for k,x in posts['parts'].items() if 'foot' in k),
        scope='single stationary-actor simulation; dense sampled 2D body versus facilities, not hardware proof')
    (run.parent/f'{run.name}_facility_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(assess(sys.argv[1]))
