"""원래 G1 판정을 보존하는 G1/G2 코스 평가. 정답은 제어기로 전달하지 않는다."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from surface_contact_eval import evaluate_run as evaluate_surface

CORE=ROOT/'artifacts/validation/2026-10-30/g2_handoff_corridor_gazebo_v1/g2_handoff_eval_core_v3.py'
spec=importlib.util.spec_from_file_location('handoff_course_core',CORE)
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)

INTEGRITY=('no_runtime_error','evaluation_readers_valid','contact_source_instrumented','sensor_only_controller_inputs',
    'sensor_only_safety_inputs','actuation_sensor_only_inputs','actor_pose_only_evaluator','drive_publisher_controller_only',
    'safe_drive_publisher_safety_only','sequential_acquisition','lidar_interface','source_unchanged','signal_sequence','no_false_start')
STOP=('model_stop_stable','dense_pose_stop_evidence','explicit_mcu_stop','no_mcu_latch_before_stop','no_deadline_diagnostic','no_wall_collision')
# 40초 진입/복귀 시험이다. 원래 full_lap은 원본 보고서에 보존한다.
COMMON=('green_observed','data_coverage','no_sampled_overlap','no_actor_contact')


def checks_for(report,runtime,surface,mode_ok,caps_ok,replay):
    """존재가 필수인 판정은 명시 목록으로 요구한다. 빈 dict의 all(True)를 피한다."""
    ev=report.get('evaluation') or {};integrity=report.get('integrity') or {};stop=report.get('stop_checks') or {}
    checks={**{'integrity_'+k:integrity.get(k) is True for k in INTEGRITY},
        **{'stop_'+k:stop.get(k) is True for k in STOP},
        **{'course_'+k:(ev.get('gates') or {}).get(k) is True for k in COMMON},
        **core.surface_ok(surface,2)}
    checks.update(harness_valid=(ev.get('harness') or {}).get('valid') is True,
        runtime_contract=runtime.get('runtime_contract',{}).get('passed') is True,
        source_unchanged=runtime.get('source_unchanged') is True,
        controller_mode=mode_ok is True,output_caps=caps_ok is True,
        no_abort=report.get('abort') is None,shutdown_clean=report.get('shutdown_clean') is True,
        replay_exact=replay.get('passed') is True and replay.get('input_unchanged') is True
            and replay.get('calls',0)>0 and replay.get('calls')==replay.get('matched'))
    additional=report.get('additional_actors') or {};actors=additional.get('actors') or {}
    checks['additional_harness']=bool(actors) and additional.get('harness_valid') is True
    combined=additional.get('combined_contacts') or {}
    checks['all_actor_contacts']=combined.get('valid') is True and combined.get('ego_events')==0
    for name,actor in actors.items():
        gates=actor.get('gates') or {};gap=actor.get('min_sampled_gap_m')
        checks['actor_'+name]=all(gates.get(k) is True for k in ('complete_gap_samples','complete_swept_intervals','no_ego_contact')) \
            and isinstance(gap,(int,float)) and gap>0
    return checks


def postreturn_progress(transitions,trace,end):
    """우회 뒤 재획득한 FOLLOW가 연속해서 진행한 거리. 재우회 진행을 섞지 않는다."""
    entries=[t['time_s'] for t in transitions if t['state']=='BYPASS']
    returns=[t['time_s'] for t in transitions if t['state']=='FOLLOW' and entries and t['time_s']>entries[0]]
    longest=0.;first=last=None
    for row in trace:
        if not returns or not returns[0]<=row['t']<=end:continue
        if (row.get('autonomy') or {}).get('handoff_state')!='FOLLOW':first=last=None;continue
        if last is None or row['t']-last['t']>.25:first=row
        last=row
        longest=max(longest,row['progress_m']-first['progress_m'])
    return bool(entries) and bool(returns),longest


def evaluate(output):
    paths=[output/p for p in ('report.json','trial_overlay.json','follow_decisions.jsonl','follow_trace_replay.json')]
    before={p.name:hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in paths}
    report=json.loads(paths[0].read_text());runtime=json.loads(paths[1].read_text());replay=json.loads(paths[3].read_text())
    green=report.get('green_observed_sim_s');end=(report.get('case_window') or {}).get('end_sim_s')
    if green is None or end is None:return dict(passed=False,error='missing drive window')
    with (output/'autonomy_status.jsonl').open() as stream:statuses=[json.loads(line) for line in stream]
    with (output/'trajectory.jsonl').open() as stream:trace=[json.loads(line) for line in stream]
    transitions=[];count=bad=0;peak=0.;first=None;last=None
    with paths[2].open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            call=event['data'];t=call['input']['reference_s'];out=call['output'];detail=out[2];state=detail.get('handoff_state')
            if t<green:continue
            count+=1;first=t if first is None else first;last=t
            cap=1. if state=='FOLLOW' else .35
            bad+=state not in ('FOLLOW','BYPASS','REACQUIRE') or not 0<=out[0]<=cap+1e-9
            peak=max(peak,out[0])
            if t<=end and (not transitions or transitions[-1]['state']!=state):
                transitions.append(dict(time_s=t,state=state,reason=detail.get('reason')))
    surface=evaluate_surface(output,start_s=green)
    cap=core.command_cap([r for r in trace if r['t']>=green],[peak] if count else [],1.)
    mode_ok=core.status_ok(statuses,'g1_g2_handoff_trial')
    checks=checks_for(report,runtime,surface,mode_ok,bool(count) and bad==0 and cap['ok'],replay)
    returned,after_progress=postreturn_progress(transitions,trace,end)
    checks['bypass_and_follow_established']=returned
    checks['postreturn_course_progress']=after_progress>=5.
    checks['raw_unchanged']=all(hashlib.file_digest(p.open('rb'),'sha256').hexdigest()==before[p.name] for p in paths)
    return dict(passed=all(checks.values()),checks=checks,evaluator='bounded_preparation_entry_return_40s_v1',
        scope='nominal sensor 40s entry and return prefix; not full-lap, hardware or default adoption',
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),CORE)},raw_sha256=before,
        original_g1=dict(passed=report.get('passed'),full_lap=(report.get('evaluation') or {}).get('gates',{}).get('full_lap'),gates=report.get('gates'),integrity=report.get('integrity'),
            partial=report.get('partial_observation_trial'),selector=report.get('pass_release_trial')),
        full_course=(report.get('evaluation') or {}).get('full_course_progress'),gap=(report.get('evaluation') or {}).get('gap'),
        transitions=transitions,postreturn_progress_m=after_progress,decision_caps=dict(count=count,violations=bad,maximum=peak,first_s=first,last_s=last,command=cap),
        surface={k:v for k,v in surface.items() if k not in ('records','rows','samples_detail')},
        replay={k:v for k,v in replay.items() if k not in ('outcomes','mismatches')})


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();result=evaluate(args.run)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:result[k] for k in ('passed','checks','transitions') if k in result}))
    raise SystemExit(0 if result['passed'] else 1)
