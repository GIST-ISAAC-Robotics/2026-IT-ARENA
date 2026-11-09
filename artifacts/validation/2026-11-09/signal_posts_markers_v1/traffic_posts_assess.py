"""시설 통합 교통 대표. 원 report 실패를 덮어쓰지 않는 사전 목표 평가."""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
from build_experimental_track import rectangle
A=Path(__file__).resolve().parent

def assess(folder):
    folder=Path(folder);run=folder/'run1';read=lambda p:json.loads(p.read_text(encoding='utf8'))
    r=read(run/'report.json');surface=read(folder/'surface.json');gap=r['evaluation']['gap']
    extra=r.get('additional_actors',{});course=r['evaluation'].get('full_course_progress',{})
    world=ET.parse(run/'runtime_world.sdf').getroot().find('world')
    fixture=ET.parse(A/'marker_posts_provisional.sdf').getroot().findall('model')
    def canonical(e):return ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True)
    models=[world.find(f"model[@name='{m.get('name')}']") for m in fixture]
    identical=all(m is not None and canonical(m)==canonical(f) for m,f in zip(models,fixture))
    meta=read(A/'marker_posts_provisional.json');signal=read(run/'signal_posts.json')
    marker_render=read(run/'course_marker_render.json')
    track=world.find("model[@name='it_arena_track_static']")
    physical_signal=[]
    for expected in signal['links']:
        link=track.find(f"link[@name='{expected['name']}']") if track is not None else None
        collision=link.find('collision') if link is not None else None
        if collision is None:physical_signal.append(False);continue
        pose=list(map(float,collision.findtext('pose').split()))
        size=list(map(float,collision.findtext('geometry/box/size').split()))
        physical_signal.append(abs(pose[0]-expected['center_xy_m'][0])<1e-6 and
            abs(pose[1]-expected['center_xy_m'][1])<1e-6 and
            abs(size[0]-expected['along_m'])<1e-6 and abs(size[1]-expected['across_m'])<1e-6)
    parts=[]
    for p in signal['links']:
        parts.append((p['name'],rectangle(*p['center_xy_m'],signal['beam_yaw_rad'],p['along_m'],p['across_m'])))
    for p in meta['links']:
        parts.append((p['name'],rectangle(*p['center'],p['yaw'],p['length'],p['width'])))
    facilities={n:dict(minimum_m=math.inf,overlap_samples=0) for n,_ in parts}
    samples=0
    with (run/'ego_poses.jsonl').open(encoding='utf8') as stream:
        for line in stream:
            t,x,y,yaw=json.loads(line);body=rectangle(x,y,yaw,.2,.15);samples+=1
            for name,p in parts:
                row=facilities[name];row['minimum_m']=min(row['minimum_m'],body.distance(p))
                row['overlap_samples']+=int(body.intersection(p).area>1e-9)
    green,end=(r['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    trace=[x for x in read(run/'trace.json') if green<=x['t']<=end]
    start=next((x['t'] for x in trace if x['ground_speed_mps']>.1),None)
    duration=longest=0.
    for a,b in zip(trace,trace[1:]):
        if start is None or a['t']<start:continue
        duration=duration+b['t']-a['t'] if a['ground_speed_mps']<.05 else 0.
        longest=max(longest,duration)
    stop=r.get('stop_checks',{})
    gates=dict(valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        one_lap=course.get('valid') is True and course.get('progress_m',0)>=44.64,
        no_contacts=gap.get('actor_contact_messages_with_ego')==0 and extra.get('combined_contacts',{}).get('valid') is True and not extra.get('combined_contacts',{}).get('events'),
        minimum_gap=gap.get('sampled_finite_complete') is True and gap.get('min_sampled_gap_m',0)>=.08,
        swept_gap=gap.get('swept_missing_actor_intervals')==0 and gap.get('min_swept_lower_bound_m') is not None and gap['min_swept_lower_bound_m']>=.08,
        additional_valid=extra.get('harness_valid') is True and extra.get('behavior_gates_passed') is True and extra.get('actor_count')==1,
        no_overlap=gap.get('sampled_overlap_samples')==0,
        surface=surface.get('samples',0)>0 and surface.get('sample_gaps_over_limit')==0 and surface.get('wall_wheel_samples')==0 and set(surface['interpretations'])=={'any','center','full'} and all(v['max_off_wheels']<=2 for v in surface['interpretations'].values()),
        stop_and_shutdown=bool(stop) and all(stop.values()) and r.get('shutdown_clean') is True,
        source_unchanged=r.get('source_unchanged') is True and read(folder/'extra_integrity.json')['changed']==[],
        process_clean=not (folder/'postflight_processes.txt').read_text().strip(),
        sustained_motion=start is not None and longest<=2.5,
        facility_xml=identical and len(physical_signal)==4 and all(physical_signal),
        marker_suppression=marker_render.get('mode')=='blank_20_30' and marker_render.get('hidden_ids')==[20,30],
        facility_clear=samples>0 and all(v['overlap_samples']==0 for v in facilities.values()))
    result=dict(passed=all(gates.values()),gates=gates,raw_pass=r['passed'],raw_gates=r['gates'],
        course=course,longest_low_speed_s=longest,facilities=facilities,dense_samples=samples,
        scope='single simulation representative, 2D body overlaps with low feet are conservative; not continuous physics proof')
    (folder/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='facilities'}));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(assess(A/'traffic_all_posts'))
