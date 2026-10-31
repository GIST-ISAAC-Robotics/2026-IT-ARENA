"""고정된 첫40초 비교. comparison_valid는 시험 무결성이며 주행 성공이 아니다."""
import argparse
import hashlib
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent
ORIGINAL=BASE.parent/'g2_motion_pause_level_v1/run1'
INTEGRITY=('no_runtime_error','evaluation_readers_valid','contact_source_instrumented','sensor_only_controller_inputs',
    'sensor_only_safety_inputs','actuation_sensor_only_inputs','actor_pose_only_evaluator','drive_publisher_controller_only',
    'safe_drive_publisher_safety_only','sequential_acquisition','lidar_interface','source_unchanged','signal_sequence','no_false_start')
STOP=('model_stop_stable','dense_pose_stop_evidence','explicit_mcu_stop','no_mcu_latch_before_stop','no_deadline_diagnostic','no_wall_collision')


def summarize(output):
    paths=[output/name for name in ('report.json','trial_overlay.json','trajectory.jsonl','follow_decisions.jsonl','follow_trace_replay.json','braking_diagnostics.jsonl')]
    digest=lambda p:hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
    hashes={p.name:digest(p) for p in paths}
    report=json.loads(paths[0].read_text());runtime=json.loads(paths[1].read_text());replay=json.loads(paths[4].read_text())
    green=report['green_observed_sim_s'];end=green+40.
    with paths[2].open() as stream:trace=[json.loads(line) for line in stream]
    trace=[r for r in trace if green<=r['t']<=end]
    diagnostic=[]
    with paths[5].open() as stream:braking=[json.loads(line) for line in stream]
    for d in braking:
        required=('sim_time_s','wheel_surface_speed_mps','truth_longitudinal_mps','truth_roll_rad')
        if all(k in d for k in required) and green<=d['sim_time_s']<=end:
            diagnostic.append(dict(t=d['sim_time_s'],wheel=d['wheel_surface_speed_mps'],truth=d['truth_longitudinal_mps'],
                difference=d['wheel_surface_speed_mps']-d['truth_longitudinal_mps'],roll=d['truth_roll_rad'],
                left=d.get('left_speed_rad_s'),right=d.get('right_speed_rad_s'),target=d.get('applied_target_mps')))
    transitions=[];resets=[];high=[];header=None
    with paths[3].open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':
                if header is None:header=event['data']
                continue
            c=event['data'];t=c['input']['reference_s']
            if not green<=t<=end:continue
            d=c['output'][2];b=d.get('bypass') or {};v=(d.get('normal') or {}).get('measured_speed_mps')
            state=d.get('handoff_state')
            if not transitions or transitions[-1]['state']!=state:
                transitions.append(dict(t=t,state=state,reason=d.get('reason')))
            if b.get('reason')=='entry_guide_input_reset':resets.append(dict(t=t,measured=v,command=c['output'][0]))
            if isinstance(v,(int,float)) and v>1:high.append(dict(t=t,measured=v,command=c['output'][0]))
    checks={**{'integrity_'+k:report.get('integrity',{}).get(k) is True for k in INTEGRITY},
            **{'stop_'+k:report.get('stop_checks',{}).get(k) is True for k in STOP}}
    checks.update(runtime=runtime.get('runtime_contract',{}).get('passed') is True,source_unchanged=runtime.get('source_unchanged') is True,
        shutdown=report.get('shutdown_clean') is True,no_abort=report.get('abort') is None,
        replay=replay.get('passed') is True and replay.get('input_unchanged') is True and replay.get('calls',0)>0 and replay.get('matched')==replay.get('calls'),
        trace_window=bool(trace) and trace[0]['t']-green<.2 and end-trace[-1]['t']<.2,
        trace_dense=bool(trace) and all(0<b['t']-a['t']<=.25 for a,b in zip(trace,trace[1:])),
        diagnostics=bool(diagnostic),raw_unchanged=all(digest(p)==hashes[p.name] for p in paths))
    return dict(comparison_evidence_valid=all(checks.values()),checks=checks,green_s=green,window_end_s=end,
        progress_first40_m=trace[-1]['progress_m']-trace[0]['progress_m'] if trace else None,
        transitions=transitions,resets=resets,measured_above_1mps=high,
        wheel_body_diagnostics=dict(samples=len(diagnostic),maximum_absolute_difference_mps=max((abs(r['difference']) for r in diagnostic),default=None),
            maximum_absolute_roll_rad=max((abs(r['roll']) for r in diagnostic),default=None),
            largest_differences=sorted(diagnostic,key=lambda r:abs(r['difference']),reverse=True)[:8]),
        original_g1_passed=report.get('passed'),original_g1_gates=report.get('gates'),
        source_sha256=runtime.get('source_sha256'),execution_config=(header or {}).get('execution_config'),
        replay={k:replay.get(k) for k in ('passed','calls','matched','input_unchanged')},raw_sha256=hashes)


def compare(output):
    baseline=summarize(ORIGINAL);variant=summarize(output)
    prefix='src/'
    sources=lambda r:{k:v for k,v in r['source_sha256'].items() if k.startswith(prefix)}
    checks=dict(baseline=baseline['comparison_evidence_valid'],variant=variant['comparison_evidence_valid'],
        same_production_sources=bool(sources(baseline)) and sources(baseline)==sources(variant),
        same_execution_config=baseline['execution_config'] is not None and baseline['execution_config']==variant['execution_config'])
    for row in (baseline,variant):row.pop('source_sha256')
    return dict(comparison_valid=all(checks.values()),checks=checks,scope='first40s paired observation; not a driving pass or causal proof',
                baseline=baseline,without_bump_collision=variant,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    result=compare(args.run)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(dict(comparison_valid=result['comparison_valid'],checks=result['checks'],
                         progress={k:result[k]['progress_first40_m'] for k in ('baseline','without_bump_collision')})))
    raise SystemExit(0 if result['comparison_valid'] else 1)
