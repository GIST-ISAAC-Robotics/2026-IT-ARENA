"""움직이는 상대차는 같은 시각의 자세로 비교한다. 정지 표적 중앙값 평가를 재사용하지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import importlib.util
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import g1_traffic_eval as ev
from surface_contact_eval import evaluate_run as surface_run
from g2_handoff_eval_core_v3 import status_ok
from arena_autonomy.follow_trace import replay_events

INTEGRITY=('no_runtime_error','evaluation_readers_valid','contact_source_instrumented','sensor_only_controller_inputs',
    'sensor_only_safety_inputs','actuation_sensor_only_inputs','actor_pose_only_evaluator','drive_publisher_controller_only',
    'safe_drive_publisher_safety_only','sequential_acquisition','lidar_interface','source_unchanged','signal_sequence','no_false_start')


def jsonl(path):
    with Path(path).open() as stream:return [json.loads(line) for line in stream]


def transition_summary(calls,start,end):
    rows=[]
    for call in calls:
        t=call['input']['reference_s'];out=call['output'];state=out[2].get('handoff_state')
        if not start<=t<=end:continue
        if not rows or rows[-1]['state']!=state:rows.append(dict(time_s=t,state=state,reason=out[2].get('reason')))
    return rows


def output_caps(calls):
    bad=[]
    for call in calls:
        speed=call['output'][0];state=call['output'][2].get('handoff_state')
        cap=1. if state=='FOLLOW' else .35
        if state not in ('FOLLOW','BYPASS','REACQUIRE') or not 0<=speed<=cap+1e-9:
            bad.append(dict(time_s=call['input']['reference_s'],state=state,speed=speed))
    return dict(passed=bool(calls) and not bad,violations=bad[:20],count=len(bad))


def evaluate(output,config,plot=None):
    output=Path(output);report=json.loads((output/'report.json').read_text())
    receipt=json.loads((output/'merge_actor_receipt.json').read_text())
    green=report.get('green_observed_sim_s');end=(report.get('case_window') or {}).get('end_sim_s')
    if green is None or end is None:return dict(passed=False,error='missing drive time window')
    ego=jsonl(output/'ego_poses.jsonl');actor=jsonl(output/'actor_poses.jsonl');statuses=jsonl(output/'autonomy_status.jsonl')
    decisions=jsonl(output/'g2_decisions.jsonl');calls=[e['data'] for e in decisions if e['kind']=='call']
    replay=replay_events(iter(decisions));replay_ok=replay['mismatches']==[]
    replay_brief={k:v for k,v in replay.items() if k not in ('outcomes','mismatches')}
    replay_brief['mismatch_count']=len(replay['mismatches']);replay_brief['passed']=replay_ok
    world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    route=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    scene=json.loads((world/'scene.json').read_text())
    centerline=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
    window=[r for r in ego if green<=r[0]<=end]
    spawn=config['launch']['g2_spawn']['s_m']
    positions=[dict(t=r[0],s=centerline.unwrap(centerline.nearest_s(r[1],r[2])[0],spawn)) for r in window]
    trigger=(receipt.get('trigger') or {}).get('elapsed_s')
    onset=None if trigger is None else green+trigger
    later=[] if onset is None else [r for r in positions if r['t']>=onset]
    post_progress=later[-1]['s']-later[0]['s'] if later else 0.
    transitions=transition_summary(calls,green,end)
    returned=bool(onset is not None and any(e['state']=='BYPASS' and e['time_s']<=onset for e in transitions)
        and any(e['state']=='FOLLOW' and e['time_s']>onset for e in transitions))
    dynamics_end=ego[-1][0]
    surface=surface_run(output,start_s=green,end_s=dynamics_end)
    surface_brief={k:dict(max_off_wheels=v['max_off_wheels'],penalty_events_merged=v['penalty_events_merged'])
                   for k,v in surface['interpretations'].items()}
    sampled=ev.sampled_gaps([r for r in ego if green<=r[0]],actor,.20,.15,.05)
    gap_valid=bool(sampled) and all(r['gap_m'] is not None for r in sampled)
    overlap=sum(r['overlap_area_m2'] is not None and r['overlap_area_m2']>1e-9 for r in sampled)
    contacts=jsonl(output/'actor_contacts.jsonl')
    hits=[t for t in ev.interval_contacts(contacts) if green<=t<=dynamics_end]
    core_eval=report.get('evaluation') or {};harness=core_eval.get('harness') or {}
    caps=output_caps(calls)
    moving_actor=[r for r in actor if onset is not None and onset<=r[0]<=end]
    displacement=(float(np.linalg.norm(np.asarray(moving_actor[-1][1:3])-np.asarray(moving_actor[0][1:3]))) if moving_actor else 0.)
    gates=dict(actor_trigger_established=trigger is not None,actor_pulse_complete=onset is not None and onset+5.5<=end,
        actor_actually_moved=displacement>.10,harness_valid=harness.get('valid') is True,
        posttrigger_progress=post_progress>=config['acceptance']['merge']['minimum_posttrigger_progress_m'],
        g1_follow_return=returned,controller_mode=status_ok(statuses,'g1_g2_handoff_trial'),output_caps=caps['passed'],
        moving_gap_complete=gap_valid,no_body_overlap=overlap==0,no_actor_contact=len(hits)==0,
        no_wall_collision=report.get('wall_collision_samples')==0,
        no_more_than_two_grass_wheels=bool(surface_brief) and all(r['max_off_wheels']<=2 for r in surface_brief.values()),
        surface_coverage=surface['samples']>0 and surface['sample_gaps_over_limit']==0,
        no_wall_wheel_samples=surface['wall_wheel_samples']==0,replay_exact=replay_ok)
    integrity=report.get('integrity') or {};stop=report.get('stop_checks') or {}
    checks={k:integrity.get(k) is True for k in INTEGRITY}
    checks.update(stop_recorded=bool(stop),**{'stop_'+k:v is True for k,v in stop.items()},
        shutdown_clean=report.get('shutdown_clean') is True,no_abort=report.get('abort') is None)
    result=dict(evaluator='g2_s_merge_v1',passed=all(gates.values()) and all(checks.values()),gates=gates,run_checks=checks,
        scope='one scripted kinematic merge in Gazebo; no stationary-target pass criterion or hardware proof',
        green_sim_s=green,window_end_sim_s=end,trigger=receipt.get('trigger'),actor_displacement_after_trigger_m=displacement,
        progress_window_m=positions[-1]['s']-positions[0]['s'] if positions else 0.,posttrigger_progress_m=post_progress,
        mode_transitions=transitions,minimum_body_gap_m=min((r['gap_m'] for r in sampled if r['gap_m'] is not None),default=None),
        overlap_samples=overlap,actor_contact_count=len(hits),surface=surface_brief,harness=harness,command_caps=caps,
        replay=replay_brief,original_g1=dict(passed=report.get('passed'),gates=report.get('gates')),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [BASE/'merge_eval.py',BASE/'g2_handoff_eval_core_v3.py']})
    return result
