"""종료된 시험의 감속·복귀·방지턱/바퀴 운동을 정렬한다. 평가 정답 전용."""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parent
GEOMETRY=BASE.parent/'g2_bump_edge_diagnosis_v1/analyze.py'
spec=importlib.util.spec_from_file_location('wheel_geometry',GEOMETRY)
geom=importlib.util.module_from_spec(spec);spec.loader.exec_module(geom)


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def analyze(run):
    names=('report.json','follow_decisions.jsonl','bump_pose_trace.json',
           'braking_diagnostics.jsonl','runtime_vehicle.sdf','runtime_world.sdf')
    hashes={name:sha(run/name) for name in names}
    report=json.loads((run/'report.json').read_text(encoding='utf-8'))
    green=report['green_observed_sim_s'];end=report['case_window']['end_sim_s']
    poses=json.loads((run/'bump_pose_trace.json').read_text(encoding='utf-8'))
    times=[r['time'] for r in poses]
    with (run/'braking_diagnostics.jsonl').open(encoding='utf-8') as stream:
        rows=[r for line in stream if green<=(r:=json.loads(line))['sim_time_s']<=end]
    vehicle=ET.parse(run/'runtime_vehicle.sdf').getroot()
    world=ET.parse(run/'runtime_world.sdf').getroot()
    link=world.find(".//model/link[@name='safety_bump_0']")
    bump=list(map(float,link.findtext('pose').split()))
    offsets={};radius=width=None
    for name in ('front_left','front_right','rear_left','rear_right'):
        wheel=vehicle.find(f".//link[@name='{name}_wheel']")
        offsets[name]=list(map(float,wheel.findtext('pose').split()))[:3]
        cylinder=wheel.find('collision/geometry/cylinder')
        rr,ww=float(cylinder.findtext('radius')),float(cylinder.findtext('length'))
        if radius is not None:assert (rr,ww)==(radius,width)
        radius,width=rr,ww
    overlap={name:None for name in offsets};closest=None;first_disagreement=None
    wheel_distance=body_distance=large_duration=0.
    for index,row in enumerate(rows):
        t=row['sim_time_s'];pose=geom.nearest(poses,times,t)
        wheels=geom.wheel_state(pose,row,offsets,bump,radius,width)
        for name,wheel in wheels.items():
            if wheel['bump_xy_projection_overlap'] and overlap[name] is None:overlap[name]=t
        distance=min(abs(w['along_m']) for w in wheels.values())
        if closest is None or distance<closest['distance_along_m']:
            closest=dict(time_s=t,distance_along_m=distance,wheels=wheels)
        error=abs(row['wheel_surface_speed_mps']-row['truth_longitudinal_mps'])
        if error>.2 and row['applied_target_mps']>.05 and first_disagreement is None:
            first_disagreement=dict(time_s=t,target_mps=row['applied_target_mps'],
                wheel_mps=row['wheel_surface_speed_mps'],body_mps=row['truth_longitudinal_mps'],wheels=wheels)
        if index:
            before=rows[index-1];dt=t-before['sim_time_s'];assert 0<dt<=.05
            wheel_distance+=.5*dt*(before['wheel_surface_speed_mps']+row['wheel_surface_speed_mps'])
            body_distance+=.5*dt*(before['truth_longitudinal_mps']+row['truth_longitudinal_mps'])
            if error>.2 and row['applied_target_mps']>.05:large_duration+=dt
    events=[];counts=Counter();previous=None;cap_calls=0
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            c=event['data'];t=c['input']['reference_s']
            if not green<=t<=end:continue
            d=c['output'][2];y=d.get('relative_yield') or {};b=d.get('bypass') or {}
            observer=y.get('current_observer') or {}
            counts[observer.get('reason','missing')]+=1
            cap_calls+=bool(y.get('cap_requested'))
            key=(d.get('handoff_state'),b.get('entry_phase'),y.get('cap_requested'),observer.get('reason'))
            if key!=previous:
                events.append(dict(t=t,since_green_s=t-green,state=key[0],phase=key[1],
                    cap_requested=key[2],observer_reason=key[3],command=c['output'][:2],
                    reason=d.get('reason'),bypass_reason=b.get('reason')))
                previous=key
    return dict(run=str(run),green_s=green,end_s=end,cap_calls=cap_calls,
        observer_counts=dict(counts),events=events,first_wheel_bump_overlap_s=overlap,
        first_commanded_large_wheel_body_difference=first_disagreement,
        commanded_large_difference_duration_s=large_duration,
        wheel_integral_m=wheel_distance,body_longitudinal_integral_m=body_distance,
        closest_bump_geometry=closest,raw_sha256=hashes,
        raw_unchanged=all(sha(run/n)==v for n,v in hashes.items()),
        scope='simulation body truth and rigid-wheel projection for diagnosis only; not measured normal loads or controller inputs')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=analyze(args.run)
    result['analysis_sources']={str(p):sha(p) for p in (Path(__file__),GEOMETRY)}
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:result[k] for k in ('cap_calls','first_wheel_bump_overlap_s',
        'commanded_large_difference_duration_s','wheel_integral_m','body_longitudinal_integral_m','raw_unchanged')}))
