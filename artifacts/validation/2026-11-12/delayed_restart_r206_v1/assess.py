"""기존 재출발 목표와 시설·형상·연결을 함께 확인한다. 원 report는 보존한다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'moving_stop_restart';RUN=D/'run1'
FAC=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-11-08/rear_face_restart_v1'),str(ROOT/'scripts'),str(ROOT/'artifacts/validation/2026-11-11/native_traffic_baseline_r87_v1')]
from goal_eval import evaluate
from build_experimental_track import rectangle


def main():
    read=lambda p:json.loads(p.read_bytes())
    evaluate(RUN,ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06')
    goal=read(D/'run1_goal.json');report=read(RUN/'report.json')
    posts=read(RUN/'dense_post_clearance.json')
    world=ET.parse(RUN/'runtime_world.sdf').getroot().find('world')
    fixture=ET.parse(FAC/'marker_posts_provisional.sdf').getroot().findall('model')
    canonical=lambda e:ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True)
    actual=[world.find(f"model[@name='{m.get('name')}']") for m in fixture]
    same=all(a is not None and canonical(a)==canonical(b) for a,b in zip(actual,fixture))
    meta=read(FAC/'marker_posts_provisional.json')
    parts=[(p['name'],rectangle(*p['center'],p['yaw'],p['length'],p['width'])) for p in meta['links']]
    clearance={n:dict(minimum_m=math.inf,overlap_samples=0) for n,_ in parts};samples=0
    for line in (RUN/'ego_poses.jsonl').open():
        t,x,y,yaw=json.loads(line);body=rectangle(x,y,yaw,.2,.15);samples+=1
        for name,p in parts:
            row=clearance[name];row['minimum_m']=min(row['minimum_m'],body.distance(p))
            row['overlap_samples']+=int(body.intersection(p).area>1e-9)
    with (RUN/'follow_decisions.jsonl').open() as stream:header=json.loads(next(stream))['data']
    sensor=ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
    actual_sensor=yaml.safe_load((RUN/'runtime_local_sensors.yaml').read_text())
    expected_sensor=yaml.safe_load(sensor.read_text())
    render=read(RUN/'course_marker_render.json')
    signal=report.get('official_signal_evidence',{})
    old_actor=ET.parse(ROOT/'artifacts/validation/2026-11-10/observed_entry_return_r63_v1/moving_stop_restart/run1/evaluation_actor.sdf').getroot()
    new_actor=ET.parse(RUN/'evaluation_actor.sdf').getroot()
    actor_collision_same=[canonical(x) for x in old_actor.findall('.//collision')]==[canonical(x) for x in new_actor.findall('.//collision')]
    from native_decision_gates import decision_gates
    delivery_gates, scan_age=decision_gates(RUN,read(RUN/'acquisition.json'))
    events=[json.loads(line) for line in (RUN/'follow_decisions.jsonl').open()]
    calls=[e['data'] for e in events if e['kind']=='call']
    candidates=[c for c in calls if c.get('delivery',{}).get('kind')=='candidate']
    delivery_gates.update(native_input=report.get('native_acquisition',{}).get('verified') is True,
        process_execution=any(e['kind']=='reset' and e['data'].get('execution')=='process_candidate' for e in events),
        all_calls_accounted=bool(calls) and all(c.get('delivery',{}).get('kind') in ('candidate','discarded') for c in calls),
        injected_delay=bool(calls) and all('injected_delay_s' in c.get('compute',{}) for c in calls),
        release_not_early=bool(calls) and all(c['compute']['released_sim_s']+1e-8>=c['input']['reference_s']+c['compute']['injected_delay_s'] for c in calls),
        planning_dt=bool(calls) and all(c['input']['dt_s']==.1 for c in calls),
        current_source_age=bool(candidates) and all(0<=c['delivery']['source_age_s']<.3 for c in candidates),
        selected_guard=header['follow_config']['guard_observation_policy']=='selected_r181',
        released_body_gap=header['follow_config']['released_body_gap_trial'] is True,
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    checks=dict(goal_passed=goal['passed'],
        signal_clear=posts['samples']>0 and len(posts['parts'])==4 and all(p['overlap_samples']==0 for p in posts['parts'].values()),
        marker_fixture_identical=same,marker_posts_clear=samples>0 and all(p['overlap_samples']==0 for p in clearance.values()),
        marker_suppression=render.get('mode')=='blank_20_30' and render.get('hidden_ids')==[20,30],
        actor_collision_unchanged=actor_collision_same,
        extra_sources_unchanged=read(D/'extra_integrity.json')['changed']==[],
        process_clean=not (D/'postflight_processes.txt').read_text().strip(),
        b0390=actual_sensor['camera']==expected_sensor['camera'] and report['local_sensor_config']['sha256']==hashlib.sha256(sensor.read_bytes()).hexdigest(),
        piecewise=header['follow_config']['piecewise_wall_trial'] is True,
        main_section=header['follow_config']['wall_offset_m']==.505,
        selected_profile=header['execution_config'].get('moving_pass_profile','slow_v1')=='medium_v1',
        actual_signal=bool(signal.get('passed') and signal.get('start_after_actual_green')))
    checks.update(delivery_gates)
    result=dict(passed=all(checks.values()), scan_age=scan_age,checks=checks,goal=goal,signal_posts=posts,
        marker_posts=clearance,dense_samples=samples,
        scope='bounded 18s combined representative; not traversal of distant marker junctions or real vehicles')
    with (D/'assessment.json').open('x') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=checks,metrics=goal['metrics']),ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
