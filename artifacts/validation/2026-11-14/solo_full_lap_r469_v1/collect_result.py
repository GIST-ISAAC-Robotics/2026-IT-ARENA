"""r469 저장 로그만 집계한다. 정책 계산·ROS·입력 수정 없음."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
R = A / 'run1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(name, value):
    (A / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pick(value, keys):
    return {key: value.get(key) for key in keys}


def ident(value):
    return value['generation'], value['sequence']


def stat(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    def percentile(q):
        i = (len(values)-1)*q
        lo = int(i)
        return values[lo] + (values[min(lo+1, len(values)-1)]-values[lo])*(i-lo)
    return dict(n=len(values), median_ms=percentile(.5), p95_ms=percentile(.95),
                max_ms=max(values), above_100ms=sum(x > 100 for x in values))


report = read(R / 'report.json')
rows = [json.loads(line) for line in (R / 'follow_decisions.jsonl').open(encoding='utf-8')]
kinds = collections.Counter(row['kind'] for row in rows)
submitted = {ident(row['data']): row for row in rows if row['kind'] == 'runtime_plan_submitted'}
completed = {ident(row['data']): row for row in rows if row['kind'] == 'runtime_plan_completed'}
calls = [row for row in rows if row['kind'] == 'call']
pubs = [row for row in rows if row['kind'] == 'route_publication']
discards = [row for row in rows if row['kind'] == 'parallel_discard']
timeline = []
transitions = []
old_mode = None
input_mismatches, output_mismatches = [], []
for row in calls:
    d = row['data']; normal = d['output'][2].get('normal') or {}
    c = d.get('current_rebase') or {}; out = c.get('output') or {}; compute = d['compute']
    key = ident(compute)
    if submitted[key]['data']['input'] != compute.get('input', d['input']):
        input_mismatches.append(row['seq'])
    if completed[key]['data']['output'] != d['output']:
        output_mismatches.append(row['seq'])
    plan_record = completed[key]['data']
    current_compute = c.get('compute') or {}
    item = dict(trace_seq=row['seq'], plan_id=list(key),
        source_first_s=normal.get('source_first_s'), reference_s=d['input']['reference_s'],
        measured_speed_mps=d['input']['speed_mps'], plan_speed_mps=d['output'][0],
        plan_reason=normal.get('reason'), selector_mode=normal.get('selector_mode'),
        selector_choice=normal.get('selector_choice'), selector_reason=normal.get('selector_reason'),
        selector_region=normal.get('selector_region'),
        plan_structure_risk=normal.get('selector_structure_risk'),
        plan_submit_trace_seq=submitted[key]['seq'], plan_complete_trace_seq=completed[key]['seq'],
        plan_wall=pick(plan_record, ('submitted_wall_s','began_wall_s','completed_wall_s','received_wall_s')),
        plan_compute_ms=1000*(plan_record['completed_wall_s']-plan_record['began_wall_s']),
        current_present=bool(c), current_result=pick(out, ('reference_s','speed_mps','reason')),
        current_id=pick(current_compute, ('generation','sequence')),
        current_wall=pick(current_compute, ('submitted_wall_s','began_wall_s','completed_wall_s','received_wall_s')),
        current_compute_ms=c.get('computation_wall_ms'),
        current_measured_speed_mps=(c.get('input') or {}).get('speed_mps'),
        current_structure_risk=(out.get('current_route_proof') or {}).get('current_structure_risk'),
        current_body_gap=out.get('current_body_gap'),
        delivery=d['delivery'])
    timeline.append(item)
    mode = normal.get('selector_mode')
    if mode != old_mode:
        transitions.append(dict(trace_seq=row['seq'], reference_s=d['input']['reference_s'],
            plan_id=list(key), old_mode=old_mode, new_mode=mode, speed_mps=d['output'][0],
            evidence=pick(normal, ('selector_mode_before','selector_reason','selector_choice',
                                   'selector_evidence','selector_sides'))))
        old_mode = mode

trajectory = [json.loads(line) for line in (R / 'trajectory.jsonl').open(encoding='utf-8')]
actuation = read(R / 'actuation_trace.json')
window = [item for item in timeline if 18.3 <= item['reference_s'] <= 19.6]
stall = [item for item in timeline if item['reference_s'] >= 19.22]
actuation_window = []
old = None
for row in actuation:
    state = row['state'], row['target_speed_mps']
    if 18.7 <= row['now_us']/1e6 <= 19.4 and state != old:
        actuation_window.append(pick(row, ('now_us','state','reason','accepted_seq',
                                          'target_speed_mps','deadline_us','feedback_seq')))
        old = state
write('timing_rows.json', dict(clock_contract='reference/at/trajectory t are simulation seconds; *_wall_s are host monotonic seconds; MCU now_us is simulation-backed MCU clock; no UTC conversion inferred',
    calls=timeline, representative_window=window,
    publications=[row for row in pubs if 18.3 <= row['data'].get('at_s', 0) <= 19.6],
    all_discards=discards,
    telemetry=[pick(row, ('t','progress_m','drive_speed_mps','safe_speed_mps','safety_reason',
                          'ground_speed_mps','mcu_state')) for row in trajectory if 18.7 <= row['t'] <= 19.4],
    mcu_transitions=actuation_window))
write('mode_transitions.json', dict(transitions=transitions, actor_count=0,
    semantic_object_identity='not recorded by selector_evidence; finite side-return count is not an actor identity; world_fixed_return is controller evidence, not simulator truth'))
summary = dict(
    original_report_passed=report['passed'], run_valid=report['run_valid'],
    behavior_gates_passed=report['behavior_gates_passed'],
    progress=report['evaluation']['full_course_progress'], metrics=report['evaluation']['case_metrics'],
    gates=report['gates'], stop_checks=report['stop_checks'], original_abort=report['abort'],
    wall_collision_samples=report['wall_collision_samples'], stop=report['stop'],
    shutdown_audit=report['shutdown_audit'], remaining_test_pids=report['remaining_test_pids'],
    diagnostics=report['actuation_diagnostics'], lidar_rate_passed=report['lidar_rate_passed'],
    source_unchanged=report['source_unchanged'], follow_kinds=dict(kinds),
    complete_trace=dict(submitted=len(submitted), completed=len(completed),
        unmatched_submitted=[list(k) for k in submitted.keys()-completed.keys()],
        unmatched_completed=[list(k) for k in completed.keys()-submitted.keys()],
        duplicate_submission_count=kinds['runtime_plan_submitted']-len(submitted),
        duplicate_completion_count=kinds['runtime_plan_completed']-len(completed),
        call_input_mismatches=input_mismatches, call_output_mismatches=output_mismatches,
        footer=next(row['data'] for row in rows if row['kind']=='footer')),
    plan_compute=stat([x['plan_compute_ms'] for x in timeline]),
    current_compute=stat([x['current_compute_ms'] for x in timeline if x['current_compute_ms'] is not None]),
    current_structure_calls_all_modes=sum(x['current_structure_risk'] is not None for x in timeline),
    route_publications=len(pubs), positive_route_publications=sum(x['data'].get('speed_mps',0)>0 for x in pubs),
    discards=dict(collections.Counter(x['data'].get('reason') for x in discards)),
    delivery_kinds=dict(collections.Counter(x['delivery'].get('kind') for x in timeline)),
    mcu_reasons=dict(collections.Counter(x['reason'] for x in actuation)),
    sustained_zero=dict(from_reference_s=19.22, calls=len(stall),
        all_plan_zero=all(x['plan_speed_mps']==0 for x in stall),
        plan_reasons=dict(collections.Counter(x['plan_reason'] for x in stall)),
        selector_reasons=dict(collections.Counter(x['selector_reason'] for x in stall)),
        modes=dict(collections.Counter(x['selector_mode'] for x in stall)),
        current_records=sum(x['current_present'] for x in stall),
        positive_publications=sum(x['data'].get('speed_mps',0)>0 and x['data'].get('at_s',0)>=19.22 for x in pubs),
        plan_compute=stat([x['plan_compute_ms'] for x in stall])))
r468_links = []
for row, item in zip(calls, timeline):
    risk = item['plan_structure_risk'] or {}
    if not risk.get('accepted'):
        continue
    following = next((p for p in pubs if p['seq'] > row['seq'] and
        p['data'].get('reference_s') == item['current_result'].get('reference_s')), None)
    r468_links.append(dict(call=item, publication=following))
summary['r468_plan_structure_accepted'] = len(r468_links)
summary['current_structure_calls_by_mode'] = dict(collections.Counter(x['selector_mode'] for x in timeline if x['current_structure_risk'] is not None))
summary['r468_released_current_structure_accepted'] = sum(x['selector_mode']=='RELEASED' and bool((x['current_structure_risk'] or {}).get('accepted')) for x in timeline)
summary['r468_positive_current'] = sum(x['call']['current_result'].get('speed_mps', 0)>0 for x in r468_links)
summary['r468_positive_publication'] = sum(x['publication'] is not None and x['publication']['data'].get('speed_mps',0)>0 for x in r468_links)
write('r468_application.json', dict(accepted_links=r468_links,
    publication_match='same current reference_s and following publication trace seq; no controller replay'))

def scalars(value):
    return {k:v for k,v in (value or {}).items() if not isinstance(v, (dict,list))}
evidence=[]
for row in calls:
    if row['seq'] not in (819,823,827,832,835):
        continue
    d=row['data']; normal=d['output'][2]['normal']; current=d.get('current_rebase') or {}
    evidence.append(dict(trace_seq=row['seq'],
        plan_input=pick(d['input'], ('reference_s','speed_mps','dt_s','epoch','previous_steering_rad','timing_verified')),
        plan_output=d['output'][:2], normal_scalars=scalars(normal),
        observation=pick(normal, ('selector_region','selector_evidence','selector_sides')),
        proof_summaries={k:scalars(normal.get(k)) for k in ('partial_observation', 'selector_rear_fringe',
            'selector_released_rear_sweep','same_path_gap','selector_released_body_gap','parallel_hold_trial')},
        current_input=pick(current.get('input') or {}, ('reference_s','speed_mps','dt_s','epoch','previous_steering_rad','timing_verified')),
        current_output_scalars=scalars(current.get('output')),
        current_body_gap=(current.get('output') or {}).get('current_body_gap'),
        current_ms=current.get('computation_wall_ms')))
write('first_stop_evidence.json', dict(source='run1/follow_decisions.jsonl; original raw fields only',
    clock_contract='simulation reference_s; computation_wall_ms is measured host elapsed, no wall/sim conversion',
    representative_calls=evidence))
write('execution_summary.json', summary)
paths = [R / name for name in ('report.json','trajectory.jsonl','follow_decisions.jsonl',
    'actuation_trace.json','actuation_diagnostics.json','stop_trace.json','source_snapshot.zip',
    'simulation.log','scans_received.jsonl','autonomy_status.jsonl')]
paths += [A / name for name in ('prepared.json','sources.zip','registered.json',
    'execution_v1/check_after_run.json','execution_v1/owned_processes_after.json',
    'execution_v1/windows_launch_v1.completed.json')]
write('raw_integrity.json', dict(files={str(p.relative_to(A)).replace('\\','/'):sha(p) for p in paths},
    original_sources_and_failure_preserved=True, additional_drive_runs=0))
print(json.dumps({k:summary[k] for k in ('progress','complete_trace','plan_compute','current_compute',
    'current_structure_calls_all_modes','discards','delivery_kinds','sustained_zero')}, ensure_ascii=True))
