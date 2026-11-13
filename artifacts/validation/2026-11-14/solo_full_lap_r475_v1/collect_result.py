"""r475 종료 후 저장 로그 집계. 정책 재계산·ROS 실행·원기록 변경 없음."""
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
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def value_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def ident(value):
    return value['generation'], value['sequence']


def stat(values):
    values = sorted(values)
    if not values:
        return {'n': 0}
    def percentile(q):
        i = (len(values) - 1) * q
        lo = int(i)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (i - lo)
    return dict(n=len(values), median_ms=percentile(.5), p95_ms=percentile(.95),
                max_ms=max(values), above_100ms=sum(x > 100 for x in values))


report = read(R / 'report.json')
trajectory = [json.loads(line) for line in (R / 'trajectory.jsonl').open(encoding='utf-8')]
actuation = read(R / 'actuation_trace.json')
green = report['green_observed_sim_s']
stop = report['stop_request_sim_s']
active = [row for row in trajectory if green <= row['t'] <= stop]
peak = max(active, key=lambda row: row['ground_speed_mps'])
milestones = {}
for distance in (9.90, 23.35, 44.64):
    row = next((row for row in active if row['progress_m'] > distance), None)
    milestones[str(distance)] = None if row is None else dict(
        t_sim_s=row['t'], elapsed_since_green_s=row['t'] - green,
        progress_m=row['progress_m'], ground_speed_mps=row['ground_speed_mps'])

kinds = collections.Counter()
submitted, completed, calls, publications = {}, {}, [], []
discards = collections.Counter()
footer = None
header = None
with (R / 'follow_decisions.jsonl').open(encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        kind, data = row['kind'], row['data']
        kinds[kind] += 1
        if kind == 'header':
            header = data
        elif kind == 'runtime_plan_submitted':
            submitted[ident(data)] = dict(seq=row['seq'], input_sha=value_sha(data['input']))
        elif kind == 'runtime_plan_completed':
            completed[ident(data)] = dict(seq=row['seq'], output_sha=value_sha(data['output']),
                compute_ms=1000 * (data['completed_wall_s'] - data['began_wall_s']))
        elif kind == 'call':
            normal = data['output'][2].get('normal') or {}
            compute = data['compute']
            current = data.get('current_rebase') or {}
            out = current.get('output') or {}
            calls.append(dict(trace_seq=row['seq'], plan_id=list(ident(compute)),
                input_sha=value_sha(compute.get('input', data['input'])), output_sha=value_sha(data['output']),
                reference_s=data['input']['reference_s'], measured_speed_mps=data['input']['speed_mps'],
                plan_speed_mps=data['output'][0], plan_reason=normal.get('reason'),
                selector_mode=normal.get('selector_mode'), selector_reason=normal.get('selector_reason'),
                current_present=bool(current), current_compute_ms=current.get('computation_wall_ms'),
                current_speed_mps=out.get('speed_mps'), current_reason=out.get('reason'),
                delivery_kind=data['delivery'].get('kind')))
        elif kind == 'route_publication':
            publications.append(dict(trace_seq=row['seq'], **data))
        elif kind == 'parallel_discard':
            discards[data.get('reason')] += 1
        elif kind == 'footer':
            footer = data

input_mismatches, output_mismatches = [], []
for row in calls:
    key = tuple(row['plan_id'])
    if submitted.get(key, {}).get('input_sha') != row['input_sha']:
        input_mismatches.append(row['trace_seq'])
    if completed.get(key, {}).get('output_sha') != row['output_sha']:
        output_mismatches.append(row['trace_seq'])
    row['plan_compute_ms'] = completed.get(key, {}).get('compute_ms')

summary = dict(
    scope='Gazebo solo actor0, facilities3, marker observer OFF; simulation evidence, not physical vehicle proof',
    original_report_passed=report['passed'], run_valid=report['run_valid'], harness_valid=report['harness_valid'],
    behavior_gates_passed=report['behavior_gates_passed'], original_abort=report['abort'],
    run_exit=int((A / 'execution_v1/run.exit').read_text()),
    progress=report['evaluation']['full_course_progress'], metrics=report['evaluation']['case_metrics'],
    milestones=milestones, solo_lap_finish_sim_s=report['solo_lap_finish_sim_s'],
    green_observed_sim_s=green, lap_elapsed_since_green_sim_s=report['solo_lap_finish_sim_s'] - green,
    speed=dict(scope='recorded Gazebo ground-speed magnitude; not commanded speed or hardware measurement',
        peak_mps=peak['ground_speed_mps'], peak_t_sim_s=peak['t'], peak_progress_m=peak['progress_m'],
        mean_definition='unchanged evaluator centerline progress / GREEN-to-STOP simulation duration',
        evaluation_window_first_sim_s=green, evaluation_window_last_sim_s=stop,
        window_first_sim_s=active[0]['t'], window_last_sim_s=active[-1]['t'], samples=len(active),
        green_to_stop_mean_progress_mps=report['evaluation']['case_metrics']['mean_progress_speed_mps']),
    gates=report['gates'], stop_checks=report['stop_checks'], stop=report['stop'],
    wall_collision_samples=report['wall_collision_samples'],
    dense_wall_overlap=report['evaluation']['dense_wall_overlap'], road=report['evaluation']['road'],
    wheel_centers=report['evaluation']['wheel_centers'],
    mcu_reasons=dict(collections.Counter(row['reason'] for row in actuation)),
    command_expired_samples=sum(row['reason'] == 'command_expired' for row in actuation),
    diagnostics=report['actuation_diagnostics'],
    shutdown_audit=report['shutdown_audit'], remaining_test_pids=report['remaining_test_pids'],
    owned_processes_after=read(A / 'execution_v1/owned_processes_after.json'),
    windows_launcher=read(A / 'execution_v1/windows_launch_v1.completed.json'),
    source_integrity=read(A / 'integrity.json'), source_unchanged=report['source_unchanged'],
    native_acquisition_verified=report['native_acquisition']['verified'], lidar_rate_passed=report['lidar_rate_passed'],
    runtime_audit=read(A / 'runtime_audit.json'),
    complete_trace=dict(kinds=dict(kinds), submitted=len(submitted), completed=len(completed), calls=len(calls),
        unmatched_submitted=[list(k) for k in submitted.keys() - completed.keys()],
        unmatched_completed=[list(k) for k in completed.keys() - submitted.keys()],
        duplicate_submission_count=kinds['runtime_plan_submitted'] - len(submitted),
        duplicate_completion_count=kinds['runtime_plan_completed'] - len(completed),
        call_input_mismatches=input_mismatches, call_output_mismatches=output_mismatches,
        comparison='SHA256 of sorted-key canonical JSON; no numeric tolerance', footer=footer),
    host_compute=dict(scope='host monotonic elapsed milliseconds under this integrated WSL load; not ARM or simulation seconds',
        all_completed_plan=stat([row['compute_ms'] for row in completed.values()]),
        call_linked_plan=stat([row['plan_compute_ms'] for row in calls if row['plan_compute_ms'] is not None]),
        current=stat([row['current_compute_ms'] for row in calls if row['current_compute_ms'] is not None])),
    delivery_kinds=dict(collections.Counter(row['delivery_kind'] for row in calls)),
    selector_modes=dict(collections.Counter(row['selector_mode'] for row in calls)),
    discards=dict(discards), route_publications=len(publications),
    positive_route_publications=sum(row.get('speed_mps', 0) > 0 for row in publications),
    wall_elapsed_s=report['wall_elapsed_s'],
    evidence_sha256={str(path.relative_to(A)): sha(path) for path in (
        A / 'prepared.json', A / 'sources.zip', A / 'root_execution_review.json',
        R / 'report.json', R / 'trajectory.jsonl', R / 'follow_decisions.jsonl', R / 'actuation_trace.json')})
write('timing_rows.json', dict(clock_contract='reference_s/trajectory t: simulation seconds; computation: host monotonic ms', calls=calls))
write('execution_summary.json', summary)
print(json.dumps({key: summary[key] for key in ('original_report_passed', 'progress', 'speed', 'command_expired_samples', 'host_compute')}))
