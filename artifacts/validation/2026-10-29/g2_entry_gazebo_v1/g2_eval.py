"""G2 가장자리 정지 표적 1회 시험의 별도 평가. 정답 자세·월드는 평가에만 사용한다.

기존 G1 판정(report.json의 passed/gates)은 그대로 보존한다. 이 평가는 g2_config_v1.json의
acceptance.g2에 실행 전에 등록한 조건만 계산한다.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
for d in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    if str(ROOT/d) not in sys.path:sys.path.insert(0,str(ROOT/d))
import g1_traffic_eval as ev
from build_experimental_track import rectangle
from surface_contact_eval import evaluate_run as surface_run


def jsonl(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def signed_lateral(centerline,x,y):
    s,_=centerline.nearest_s(x,y)
    # 최근접 꼭짓점 주변에서 국소 선분 투영으로 부호와 진행 방향을 얻는다.
    best=None
    for ds in np.linspace(-.1,.1,41):
        cx,cy,yaw=centerline.pose(s+ds)
        along=(x-cx)*math.cos(yaw)+(y-cy)*math.sin(yaw)
        if best is None or abs(along)<abs(best[0]):
            best=(along,s+ds,-(x-cx)*math.sin(yaw)+(y-cy)*math.cos(yaw),yaw)
    return best[1],best[2],best[3]


def corners_s(centerline,reference,x,y,yaw):
    values=[]
    for cx,cy in rectangle(x,y,yaw,.20,.15).exterior.coords[:-1]:
        s,_,_=signed_lateral(centerline,cx,cy)
        values.append(centerline.unwrap(s,reference))
    return min(values),max(values)


def replay(path):
    from arena_autonomy.follow_trace import replay_events
    from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
    events=jsonl(path)
    header=events[0]['data']
    report=replay_events(events)
    reasons={}
    for row in report['outcomes']:
        reason=row['details'].get('reason');reasons[reason]=reasons.get(reason,0)+1
    phases=[(row['reference_s'],row['details'].get('entry_phase')) for row in report['outcomes']]
    first={}
    for t,phase in phases:
        if phase is not None and phase not in first:first[phase]=t
    return dict(passed=report['passed'],calls=report['calls'],matched=report['matched'],mismatches=len(report['mismatches']),
        header_schema=header.get('schema'),controller_mode=header.get('controller_mode'),
        execution_config_matches_default=header.get('execution_config')==asdict(BypassExecutionConfig()),
        follow_config={k:header['follow_config'].get(k) for k in ('partial_observation_policy','boundary_evidence_mode',
            'occupancy_timing_mode','corridor_fallback_enabled','max_speed_mps','max_steering_rad','wheelbase_m',
            'wall_offset_m','lateral_acceleration_mps2')},
        reason_counts=dict(sorted(reasons.items(),key=lambda kv:-kv[1])),first_phase_reference_s=first,
        positive_commands=sum(row['speed_mps']>0 for row in report['outcomes']),
        maximum_command_mps=max((row['speed_mps'] for row in report['outcomes']),default=None))


def evaluate(output,config):
    output=Path(output);rules=config['acceptance']['g2']
    report=json.loads((output/'report.json').read_text(encoding='utf-8'))
    world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    route=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    scene=json.loads((world/'scene.json').read_text(encoding='utf-8'))
    centerline=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
    spawn=config['launch']['g2_spawn']
    ego=[tuple(r) for r in jsonl(output/'ego_poses.jsonl')]
    actor=[tuple(r) for r in jsonl(output/'actor_poses.jsonl')]
    contacts=jsonl(output/'actor_contacts.jsonl')
    statuses=jsonl(output/'autonomy_status.jsonl')
    trace=json.loads((output/'trace.json').read_text(encoding='utf-8'))
    stop_trace=json.loads((output/'stop_trace.json').read_text(encoding='utf-8')) if (output/'stop_trace.json').exists() else []
    green=report.get('green_observed_sim_s');window_end=(report.get('case_window') or {}).get('end_sim_s')
    stop_end=stop_trace[-1]['time'] if stop_trace else window_end
    result=dict(scope='G2 single registered run evaluation; truth poses/world are evaluation-only',rules=rules,
                green_sim_s=green,window_end_sim_s=window_end,stop_end_sim_s=stop_end)
    if green is None or window_end is None or not ego or not actor:
        result.update(passed=False,error='missing green/window/ego/actor evidence')
        return result
    ax,ay,ayaw=np.median(np.asarray([a[1:] for a in actor]),axis=0)
    actor_box=rectangle(ax,ay,ayaw,.20,.15)
    actor_lo,actor_hi=corners_s(centerline,spawn['s_m'],ax,ay,ayaw)
    rows=[]
    for t,x,y,yaw in ego:
        if t<green or t>stop_end:continue
        s,lateral,road_yaw=signed_lateral(centerline,x,y)
        lo,hi=corners_s(centerline,spawn['s_m'],x,y,yaw)
        body=rectangle(x,y,yaw,.20,.15)
        rows.append(dict(t=t,s=centerline.unwrap(s,spawn['s_m']),lateral=lateral,heading=ev.wrap_angle(yaw-road_yaw),
                         rear_s=lo,front_s=hi,gap=float(body.distance(actor_box)),overlap=body.intersects(actor_box)))
    passed_rows=[r for r in rows if r['rear_s']>actor_hi and r['t']<=window_end]
    pass_t=passed_rows[0]['t'] if passed_rows else None
    after=[r for r in rows if pass_t is not None and r['t']>=pass_t]
    ok=lambda r:abs(r['lateral'])<=rules['returned_to_center_lateral_m'] and abs(r['heading'])<=rules['returned_to_center_heading_rad']
    best=start=first_center=None;longest=0.
    for r in after:
        if ok(r):
            start=r['t'] if start is None else start
            if r['t']-start>longest:longest=r['t']-start
            if longest>=rules['returned_to_center_hold_s'] and first_center is None:first_center=start
        else:start=None
    hits=[t for t in ev.interval_contacts(contacts) if green<=t<=stop_end]
    surface=surface_run(output,start_s=green,end_s=stop_end)
    interpretations={k:dict(max_off_wheels=v['max_off_wheels'],penalty_events_merged=v['penalty_events_merged'])
                     for k,v in surface['interpretations'].items()}
    drive=[r for r in trace if r.get('elapsed_since_green_s') is not None]
    g2_rows=[s for s in statuses if s.get('behavior') is not None]
    lateral_peak=max(rows,key=lambda r:abs(r['lateral'])) if rows else None
    result.update(
        target=dict(x=float(ax),y=float(ay),yaw=float(ayaw),front_s=actor_hi,rear_s=actor_lo,pose_samples=len(actor)),
        ego_samples=len(rows),progress_m=(rows[-1]['s']-rows[0]['s']) if rows else 0.,
        pass_time_since_green_s=None if pass_t is None else pass_t-green,
        maximum_lateral_m=None if lateral_peak is None else lateral_peak['lateral'],
        maximum_lateral_time_since_green_s=None if lateral_peak is None else lateral_peak['t']-green,
        center_return_first_time_since_green_s=None if first_center is None else first_center-green,
        longest_centered_after_pass_s=longest,
        final_lateral_m=rows[-1]['lateral'] if rows else None,final_heading_rad=rows[-1]['heading'] if rows else None,
        minimum_body_gap_m=min((r['gap'] for r in rows),default=None),overlap_samples=sum(r['overlap'] for r in rows),
        actor_contact_messages_with_ego=len(hits),
        harness_gap=(report.get('evaluation') or {}).get('gap'),
        surface=dict(interpretations=interpretations,wall_wheel_samples=surface['wall_wheel_samples'],
                     samples=surface['samples'],evaluator=surface['evaluator_version'],steering_source=surface['steering_source']),
        maximum_drive_mps=max((r['drive_speed_mps'] for r in drive),default=None),
        maximum_safe_mps=max((r['safe_speed_mps'] for r in drive),default=None),
        status_rows=len(g2_rows),status_mode_g2=bool(g2_rows) and all(s.get('mode')=='g2_entry_trial' and
            s.get('simulation_trial_only') is True and s.get('automatic_race_mode_selection') is False for s in g2_rows),
        status_phases=sorted({s.get('entry_phase') for s in g2_rows},key=str),
        status_reasons={k:sum(s.get('reason')==k for s in g2_rows) for k in sorted({s.get('reason') for s in g2_rows},key=str)})
    integrity=report.get('integrity',{});stop=report.get('stop_checks',{})
    integrity_keys=('no_runtime_error','evaluation_readers_valid','contact_source_instrumented','sensor_only_controller_inputs',
        'sensor_only_safety_inputs','actuation_sensor_only_inputs','actor_pose_only_evaluator','drive_publisher_controller_only',
        'safe_drive_publisher_safety_only','sequential_acquisition','lidar_interface','source_unchanged','signal_sequence','no_false_start')
    gates=dict(
        target_passed=pass_t is not None,
        returned_to_center=first_center is not None,
        no_actor_contact=len(hits)<=rules['actor_contact_messages_with_ego'],
        no_body_overlap=result['overlap_samples']<=rules['overlap_samples'],
        wheels_two_or_fewer_off_road=all(v['max_off_wheels']<=rules['max_off_road_wheels'] for v in interpretations.values()),
        no_wall_wheel=surface['wall_wheel_samples']<=rules['wall_wheel_samples'],
        command_cap=result['maximum_drive_mps'] is not None and result['maximum_drive_mps']<=rules['command_cap_mps']+1e-9,
        g2_controller_rows=result['status_mode_g2'])
    run_checks={k:integrity.get(k) is True for k in integrity_keys}
    run_checks.update({f'stop_{k}':v is True for k,v in stop.items()},shutdown_clean=report.get('shutdown_clean') is True,
                      no_abort=report.get('abort') is None)
    result.update(gates=gates,run_checks=run_checks,original_g1=dict(passed=report.get('passed'),gates=report.get('gates'),
        integrity_controller_mode_matches=integrity.get('controller_mode_matches'),
        scope='preserved G1 stopped_center verdict; HOLD/stay-behind/body-road-only are not G2 gates'))
    trace_path=output/'g2_decisions.jsonl'
    try:
        result['replay']=replay(trace_path)
    except Exception as error:
        result['replay']=dict(passed=False,error=f'{type(error).__name__}: {error}')
    result['passed']=bool(all(gates.values()) and all(run_checks.values()) and result['replay'].get('passed') is True
                          and result['replay'].get('header_schema')==4)
    try:
        import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(4,9))
        ax.plot([r['lateral'] for r in rows],[r['s']-spawn['s_m'] for r in rows],lw=1.2,label='ego centre (truth)')
        _,tl,_=signed_lateral(centerline,float(result['target']['x']),float(result['target']['y']))
        ax.add_patch(plt.Rectangle((tl-.075,actor_lo-spawn['s_m']),.15,actor_hi-actor_lo,color='tab:red',alpha=.5,label='target'))
        for edge in (-.225,.225):ax.axvline(edge,color='k',lw=.8)
        for edge in (-.425,.425):ax.axvline(edge,color='g',lw=.8)
        ax.set_xlabel('lateral from centerline [m] (+left)');ax.set_ylabel('along road from start [m]')
        ax.set_xlim(-.45,.45);ax.legend(loc='upper right',fontsize=7);ax.set_title('G2 Gazebo run (evaluation only)',fontsize=9)
        fig.tight_layout();fig.savefig(output.parent/(output.name+'_path.png'),dpi=120);plt.close(fig)
        result['plot']=output.name+'_path.png'
    except Exception as error:
        result['plot_error']=f'{type(error).__name__}: {error}'
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('output',type=Path);parser.add_argument('--result',type=Path,required=True)
    args=parser.parse_args()
    config=json.loads((BASE/'g2_config_v1.json').read_text(encoding='utf-8'))
    result=evaluate(args.output.resolve(),config)
    with args.result.open('x',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=1))
    print(json.dumps(dict(passed=result['passed'],gates=result.get('gates'),run_checks=result.get('run_checks')),ensure_ascii=False))
