"""좌우 피추월의 초기 배치·자차 횡이동/방향·관측/발행 시간을 분리하는 사후 평가.

정답/중심선은 이 평가에만 사용한다. 고정 횡위치 비교는 같은 종방향 표본을 쓰는
기하 분해일 뿐 실제 다른 주행 궤적이나 회피 가능성의 증명이 아니다.
"""
import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import g1_traffic_eval as ev


def read(p):return json.loads(p.read_text(encoding='utf-8'))


def one(name):
    folder=ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run'
    route_path=ROOT/'src/arena_gazebo/worlds/it_arena_official/centerline.csv'
    scene_path=route_path.with_name('scene.json')
    inputs=[folder/f for f in ('report.json','ego_poses.jsonl','additional_actors.json','follow_decisions.jsonl')]+[route_path,scene_path]
    input_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    report=read(folder/'report.json')
    for p in (route_path,scene_path):
        expected=report['source_sha256'][p.relative_to(ROOT).as_posix()]
        assert hashlib.sha256(p.read_bytes()).hexdigest()==expected, f'historical world differs: {p}'
    route=np.genfromtxt(route_path,delimiter=',',names=True)
    center=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],read(scene_path)['track']['lap_length_m'])
    ego=[json.loads(l) for l in (folder/'ego_poses.jsonl').open(encoding='utf-8')]
    actor=read(folder/'additional_actors.json')['poses']['eval_e3_side']
    times=[r[0] for r in actor]
    first=report['case_window']['green_sim_s'];end=min(first+3.,report['case_window']['end_sim_s'])
    rows=[];d0=None;miss=0
    for t,x,y,yaw in ego:
        if not first<=t<=end:continue
        i=bisect_right(times,t)-1
        if i<0 or t-times[i]>ev.POSE_MAX_GAP_S:miss+=1;continue
        _,ax,ay,ayaw=actor[i]
        along,_=center.nearest_s(x,y);cx,cy,heading=center.pose(along)
        lateral=-(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading)
        if d0 is None:d0=lateral
        fixed_x=cx-d0*math.sin(heading);fixed_y=cy+d0*math.cos(heading)
        obstacle=ev.rectangle(ax,ay,ayaw,.20,.15)
        gap=ev.rectangle(x,y,yaw,.20,.15).distance(obstacle)
        aligned=ev.rectangle(x,y,heading,.20,.15).distance(obstacle)
        frozen=ev.rectangle(fixed_x,fixed_y,heading,.20,.15).distance(obstacle)
        rows.append({'t':t,'actual_gap_m':gap,'same_center_road_heading_gap_m':aligned,
            'fixed_initial_lateral_gap_m':frozen,'yaw_gap_effect_m':gap-aligned,
            'lateral_projection_gap_effect_m':aligned-frozen,'lateral_m':lateral,
            'yaw_error_rad':math.remainder(yaw-heading,math.tau)})
    calls=[]
    for line in (folder/'follow_decisions.jsonl').open(encoding='utf-8'):
        event=json.loads(line)
        if event['kind']!='call':continue
        data=event['data'];t=data['input']['reference_s']
        if not first<=t<=end:continue
        output=data['output'];lat=output[2].get('m3_lateral') or {}
        yield_check=(lat.get('checks') or {}).get('yield')
        rejection=[]
        if yield_check:
            rejection=list(yield_check.get('reasons') or [])
            cmd=yield_check.get('command') or {}
            if cmd and not cmd.get('accepted'):rejection.append(cmd.get('reason'))
        calls.append({'t':t,'side_present':any(v.get('present') for v in (lat.get('threats') or {}).values()),
            'mode':lat.get('mode'),'target':output[0],'steering':output[1],'reason':output[2].get('reason'),
            'yield_evaluated':yield_check is not None,'yield_rejection':rejection,
            'yield_plan_margin_m':None if yield_check is None else yield_check.get('lateral_min_margin_m')})
    first_threat=next((c['t'] for c in calls if c['side_present']),None)
    thresholds={}
    for gap in [.15,.115,.097]:
        crossing=next((r for r in rows if r['actual_gap_m']<gap),None)
        thresholds[str(gap)]={'first_below_s':None if crossing is None else crossing['t'],
            'time_after_first_threat_s':None if crossing is None or first_threat is None else crossing['t']-first_threat}
    at_min=min(rows,key=lambda r:r['actual_gap_m'])
    min_time=at_min['t']
    before=[c for c in calls if c['t']<=min_time]
    rejected=Counter(reason for c in before for reason in c['yield_rejection'])
    summary={'window_s':[first,end],'samples':len(rows),'unmatched_ego_samples':miss,
        'actual_minimum':at_min,'fixed_lateral_minimum_m':min(r['fixed_initial_lateral_gap_m'] for r in rows),
        'max_abs_lateral_change_before_min_m':max(abs(r['lateral_m']-d0) for r in rows if r['t']<=min_time),
        'first_side_threat_s':first_threat,'threshold_crossings':thresholds,
        'first_positive_yield_s':next((c['t'] for c in before if c['mode']=='YIELD_TRIAL' and c['target']>0),None),
        'yield_evaluated_before_min_calls':sum(c['yield_evaluated'] for c in before),
        'yield_rejection_before_min':dict(rejected),
        'decomposition_identity_max_error':max(abs(r['actual_gap_m']-r['fixed_initial_lateral_gap_m']-
             r['yaw_gap_effect_m']-r['lateral_projection_gap_effect_m']) for r in rows),
        'original_verdict_passed':report['passed']}
    return summary,{'geometry':rows,'decisions':calls},input_hashes


def main(out):
    if out.exists():raise FileExistsError(out)
    source=[Path(__file__),ROOT/'scripts/g1_traffic_eval.py']
    before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source}
    results={};rows={};inputs={}
    for name in ['run17_observed_release_closed_loop','run21_right_encounter']:
        results[name],rows[name],hashes=one(name);inputs.update(hashes)
        print(json.dumps({name:results[name]}),flush=True)
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in {**before,**inputs}.items())
    out.mkdir(parents=True)
    report={'scope':'posthoc geometry decomposition only; controller is not replayed or given truth',
        'source_sha256':before,'input_sha256':inputs,'source_and_input_unchanged':unchanged,'results':results,
        'limits':['Fixed-lateral counterfactual is not a feasible closed-loop trajectory.',
                  'Actor poses are set_pose updates; step-size / short interval is not measured vehicle speed.',
                  'Thresholds characterize existing budgets; no old verdict or acceptance line changed.']}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (out/'rows.json').write_text(json.dumps(rows)+'\n',encoding='utf-8')
    return 0 if unchanged and all(r['unmatched_ego_samples']==0 and r['decomposition_identity_max_error']<1e-10 for r in results.values()) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);sys.exit(main(p.parse_args().output))
