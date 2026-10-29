"""G2 정지 표적 시험 평가 v2(주행 창/STOP 창 분리). 정답 자세·월드는 평가에만 사용한다.

판정 계산은 g2_eval_core.py에 두고, 여기서는 저장 기록을 읽어 행을 만든다. 기존 g2_entry_gazebo_v1/g2_eval.py와
run1_g2_evaluation.json은 수정하지 않는다. 그 파일의 기하 보조 함수(signed_lateral·corners_s·replay)는 그대로 불러 쓴다.
기존 G1 판정(report.json)은 original_g1에 보존만 한다.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
RUN1=BASE.parent/'g2_entry_gazebo_v1'
for d in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    if str(ROOT/d) not in sys.path:sys.path.insert(0,str(ROOT/d))
sys.path.insert(0,str(BASE))
import g1_traffic_eval as ev
from build_experimental_track import rectangle
from surface_contact_eval import evaluate_run as surface_run
import g2_eval_core as core

_spec=importlib.util.spec_from_file_location('g2_eval_v1_helpers',RUN1/'g2_eval.py')
v1=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(v1)
VERSION='g1_g2_handoff_eval_v1'
INTEGRITY_KEYS=('no_runtime_error','evaluation_readers_valid','contact_source_instrumented','sensor_only_controller_inputs',
    'sensor_only_safety_inputs','actuation_sensor_only_inputs','actor_pose_only_evaluator','drive_publisher_controller_only',
    'safe_drive_publisher_safety_only','sequential_acquisition','lidar_interface','source_unchanged','signal_sequence','no_false_start')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evaluate(output,config,rules=None,plot=None):
    """config: 시험 배치(case·spawn·harness). rules: acceptance.g2(기본은 config 안의 값)."""
    output=Path(output);rules=rules or config['acceptance']['g2'];case=config['cases'][0]
    report=json.loads((output/'report.json').read_text(encoding='utf-8'))
    world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    route=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    scene=json.loads((world/'scene.json').read_text(encoding='utf-8'))
    centerline=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
    spawn=config['launch']['g2_spawn']
    ego=[tuple(r) for r in v1.jsonl(output/'ego_poses.jsonl')]
    actor=[tuple(r) for r in v1.jsonl(output/'actor_poses.jsonl')]
    contacts=v1.jsonl(output/'actor_contacts.jsonl')
    statuses=v1.jsonl(output/'autonomy_status.jsonl')
    trace=json.loads((output/'trace.json').read_text(encoding='utf-8'))
    stop_trace=json.loads((output/'stop_trace.json').read_text(encoding='utf-8')) if (output/'stop_trace.json').exists() else []
    green=report.get('green_observed_sim_s');window_end=(report.get('case_window') or {}).get('end_sim_s')
    stop_end=stop_trace[-1]['time'] if stop_trace else None
    result=dict(evaluator=VERSION,core=core.CORE_VERSION,
                sources={p:sha(BASE/p) for p in ('g2_eval_v2.py','g2_eval_core.py')}|{'g2_entry_gazebo_v1/g2_eval.py':sha(RUN1/'g2_eval.py')},
                scope='G2 stationary-target evaluation v2; truth poses/world are evaluation-only',rules=rules,case=case['name'],
                green_sim_s=green,window_end_sim_s=window_end,stop_end_sim_s=stop_end,
                window_note='return/hold judged only in [first pass, window_end]; STOP window (window_end, stop_end] reported and gated separately')
    if green is None or window_end is None or stop_end is None or not ego or not actor:
        result.update(passed=False,error='missing green/window/stop/ego/actor evidence');return result
    ax,ay,ayaw=np.median(np.asarray([a[1:] for a in actor]),axis=0)
    target_static=core.static_target(actor,(ax,ay),green,stop_end,config['acceptance']['harness']['actor_static_error_max_m'],
                                     case.get('schedule'))
    actor_box=rectangle(ax,ay,ayaw,.20,.15)
    actor_lo,actor_hi=v1.corners_s(centerline,spawn['s_m'],ax,ay,ayaw)
    _,target_lateral,_=v1.signed_lateral(centerline,float(ax),float(ay))
    rows=[];ego_times=[]
    for t,x,y,yaw in ego:
        if t<green or t>stop_end:continue
        ego_times.append(t)
        s,lateral,road_yaw=v1.signed_lateral(centerline,x,y)
        lo,hi=v1.corners_s(centerline,spawn['s_m'],x,y,yaw)
        body=rectangle(x,y,yaw,.20,.15)
        rows.append(dict(t=t,s=centerline.unwrap(s,spawn['s_m']),lateral=lateral,heading=ev.wrap_angle(yaw-road_yaw),
                         rear_s=lo,front_s=hi,gap=float(body.distance(actor_box)),overlap=bool(body.intersects(actor_box))))
    hits=[t for t in ev.interval_contacts(contacts) if green<=t<=stop_end]
    surface=surface_run(output,start_s=green,end_s=stop_end)
    surface_brief=dict(interpretations={k:dict(max_off_wheels=v['max_off_wheels'],penalty_events_merged=v['penalty_events_merged'],
                                               time_by_off_wheel_count_s=v['time_by_off_wheel_count_s'])
                                        for k,v in surface['interpretations'].items()},
                       wall_wheel_samples=surface['wall_wheel_samples'],samples=surface['samples'],
                       sample_gaps_over_limit=surface['sample_gaps_over_limit'],evaluator=surface['evaluator_version'],
                       steering_source=surface['steering_source'],
                       steering_limit=('contact patches use the MCU target central steering, not measured wheel steering; '
                                       'they are a planned/target-geometry range, not a measured footprint'))
    try:
        replay=v1.replay(output/'g2_decisions.jsonl')
    except Exception as error:
        replay=dict(passed=False,error=f'{type(error).__name__}: {error}')
    with (output/'g2_decisions.jsonl').open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data']
    decision_speeds=[replay['maximum_command_mps']] if replay.get('maximum_command_mps') is not None else []
    harness_valid=report.get('harness_valid') is True and ((report.get('evaluation') or {}).get('harness') or {}).get('valid') is True
    gates,detail=core.gates(rows=rows,green=green,window_end=window_end,stop_end=stop_end,target_front_s=actor_hi,
        target_lateral_m=case['actor_lateral_offset_m'],rules=rules,ego_times=ego_times,contacts_in_run=len(hits),
        overlap_samples=sum(r['overlap'] for r in rows),surface=surface_brief,trace_rows=trace,decision_speeds=decision_speeds,
        header=header,replay=replay,status_rows=statuses,harness_valid=harness_valid,target_static=target_static)
    gates['target_lateral_sign_matches_case']=bool(np.sign(target_lateral)==np.sign(case['actor_lateral_offset_m']))
    window=[r for r in rows if r['t']<=window_end]
    peak=max(window,key=lambda r:abs(r['lateral'])) if window else None
    g2_rows=[s for s in statuses if s.get('behavior') is not None]
    integrity=report.get('integrity',{});stop=report.get('stop_checks',{})
    run_checks={k:integrity.get(k) is True for k in INTEGRITY_KEYS}
    run_checks.update({f'stop_{k}':v is True for k,v in stop.items()},shutdown_clean=report.get('shutdown_clean') is True,
                      no_abort=report.get('abort') is None)
    rel=lambda t:None if t is None else t-green
    result.update(
        target=dict(x=float(ax),y=float(ay),yaw=float(ayaw),signed_lateral_m=float(target_lateral),front_s=actor_hi,rear_s=actor_lo,
                    pose_samples=len(actor),static=target_static),
        ego_samples=len(rows),
        progress_window_m=(window[-1]['s']-rows[0]['s']) if window else 0.,
        progress_to_stop_end_m=rows[-1]['s']-rows[0]['s'],
        pass_time_since_green_s=rel(detail['pass_time_s']),
        bypass_side=detail['bypass_side'],
        maximum_lateral_in_window_m=None if peak is None else peak['lateral'],
        maximum_lateral_time_since_green_s=None if peak is None else rel(peak['t']),
        center_in_window=dict(detail['center'],first_start_since_green_s=rel(detail['center']['first_start_s']),
                              hold_satisfied_since_green_s=rel(detail['center']['hold_satisfied_s'])),
        stop_window=detail['stop_window'],
        lateral_at_window_end_m=window[-1]['lateral'] if window else None,
        final_lateral_m=rows[-1]['lateral'],final_heading_rad=rows[-1]['heading'],
        minimum_body_gap_m=min(r['gap'] for r in rows),overlap_samples=sum(r['overlap'] for r in rows),
        actor_contact_messages_with_ego=len(hits),harness_gap=(report.get('evaluation') or {}).get('gap'),
        coverage=dict(window=detail['window_coverage'],stop=detail['stop_coverage']),
        surface=surface_brief,command_cap=detail['command_cap'],trace_header=detail['trace_header'],
        status_rows=len(g2_rows),status_phases=sorted({s.get('entry_phase') for s in g2_rows},key=str),
        status_reasons={k:sum(s.get('reason')==k for s in g2_rows) for k in sorted({s.get('reason') for s in g2_rows},key=str)},
        gates=gates,run_checks=run_checks,replay=replay,
        original_g1=dict(passed=report.get('passed'),gates=report.get('gates'),
                         integrity_controller_mode_matches=integrity.get('controller_mode_matches'),
                         scope='preserved G1 stopped_center verdict; HOLD/stay-behind/body-road-only are not G2 gates'))
    from arena_autonomy.follow_trace import replay_events
    with (output/'g2_decisions.jsonl').open(encoding='utf-8') as stream:
        full_replay=replay_events(json.loads(line) for line in stream)
    decisions=[r for r in full_replay['outcomes'] if green<=r['reference_s']<=window_end]
    entries=[r for r in decisions if r['details'].get('reason')=='handoff_entry']
    returns=[r for r in decisions if r['details'].get('reason')=='handoff_return']
    gates['automatic_entry_observed']=bool(entries)
    gates['automatic_return_after_entry']=bool(entries and returns and returns[0]['reference_s']>entries[0]['reference_s'])
    gates['first_output_follows']=bool(decisions) and decisions[0]['details'].get('handoff_state')=='FOLLOW'
    bypass_rows=[r for r in decisions if r['details'].get('handoff_state') in ('BYPASS','REACQUIRE')]
    gates['bypass_cap_035']=bool(bypass_rows) and max(r['speed_mps'] for r in bypass_rows)<=.35+1e-9
    result['handoff']=dict(entry_times_since_green_s=[r['reference_s']-green for r in entries],
        return_times_since_green_s=[r['reference_s']-green for r in returns],
        state_counts={state:sum(r['details'].get('handoff_state')==state for r in decisions)
                      for state in ('FOLLOW','BYPASS','REACQUIRE')},
        full_decision_replay=full_replay['passed'])
    result['passed']=bool(all(gates.values()) and all(run_checks.values()))
    if plot:
        try:
            import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
            fig,axis=plt.subplots(figsize=(4,9))
            axis.plot([r['lateral'] for r in window],[r['s']-spawn['s_m'] for r in window],lw=1.2,label='ego centre, drive window')
            tail=[r for r in rows if r['t']>window_end]
            axis.plot([r['lateral'] for r in tail],[r['s']-spawn['s_m'] for r in tail],lw=1.2,color='tab:gray',label='STOP window')
            axis.add_patch(plt.Rectangle((target_lateral-.075,actor_lo-spawn['s_m']),.15,actor_hi-actor_lo,color='tab:red',alpha=.5,
                                         label='target'))
            for edge in (-.225,.225):axis.axvline(edge,color='k',lw=.8)
            for edge in (-.425,.425):axis.axvline(edge,color='g',lw=.8)
            axis.set_xlim(.45,-.45)   # 진행 방향을 위로 볼 때 왼쪽(+)이 그림 왼쪽
            axis.set_xlabel('lateral from centerline [m] (+left, drawn on the left)');axis.set_ylabel('along road from start [m]')
            axis.legend(loc='upper right',fontsize=7);axis.set_title(f'G2 Gazebo {case["name"]} (evaluation only)',fontsize=8)
            fig.tight_layout();fig.savefig(plot,dpi=120);plt.close(fig);result['plot']=str(Path(plot).name)
        except Exception as error:
            result['plot_error']=f'{type(error).__name__}: {error}'
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('output',type=Path);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--rules-from',type=Path,help='acceptance.g2 rules from another config (post-hoc audit only)')
    parser.add_argument('--result',type=Path,required=True);parser.add_argument('--plot',type=Path)
    parser.add_argument('--label',default='')
    args=parser.parse_args()
    config=json.loads(args.config.read_text(encoding='utf-8'))
    rules=json.loads(args.rules_from.read_text(encoding='utf-8'))['acceptance']['g2'] if args.rules_from else None
    result=evaluate(args.output.resolve(),config,rules,args.plot)
    if args.label:result['label']=args.label
    with args.result.open('x',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=1))
    print(json.dumps(dict(passed=result['passed'],gates=result.get('gates'),failed_run_checks=[k for k,v in (result.get('run_checks') or {}).items() if not v]),ensure_ascii=False))
