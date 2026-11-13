"""r452 원자료 집계만 수행하며 판정/입력/제어기는 변경하지 않는다."""
import collections
import hashlib
import json
from pathlib import Path

B = Path(__file__).resolve().parent
P = B / 'run1'
O = B / 'analysis_v1'
O.mkdir(exist_ok=True)
read = lambda p: json.loads(p.read_bytes())
lines = lambda p: [json.loads(s) for s in p.read_text(encoding='utf-8').splitlines() if s]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
write = lambda n, v: (O / n).write_text(json.dumps(v, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
r = read(P / 'report.json')
t = lines(P / 'follow_decisions.jsonl')
by = lambda k: [e for e in t if e['kind'] == k]
sub, done, calls, pubs = map(by, ['runtime_plan_submitted', 'runtime_plan_completed', 'call', 'route_publication'])
key = lambda e: (e['data']['generation'], e['data']['sequence'])
sm, dm = {key(e): e for e in sub}, {key(e): e for e in done}
normal = lambda e: e['data']['output'][2].get('normal', {})
traj = lines(P / 'trajectory.jsonl')
act = read(P / 'actuation_trace.json')
curr = [e for e in calls if e['data'].get('current_rebase')]
stalled = [e for e in sub if e['data']['input']['reference_s'] >= 34.999]
st_done = [dm[key(e)] for e in stalled if key(e) in dm]
counts = lambda seq: dict(collections.Counter(seq))

# Continuous command episode from saved trajectory samples; endpoint is first
# following non-.1 sample, not an inferred sensor publication timestamp.
episodes = []
i = 0
while i < len(traj):
    if abs(traj[i]['drive_speed_mps'] - .1) > 1e-6:
        i += 1
        continue
    first = i
    while i + 1 < len(traj) and abs(traj[i + 1]['drive_speed_mps'] - .1) <= 1e-6:
        i += 1
    last = i
    following = traj[i + 1:] if i + 1 < len(traj) else []
    e = dict(first_sim_s=traj[first]['t'], last_sim_s=traj[last]['t'],
             sampled_duration_s=traj[last]['t'] - traj[first]['t'],
             progress_start_m=traj[first]['progress_m'], progress_end_m=traj[last]['progress_m'],
             sampled_progress_m=traj[last]['progress_m'] - traj[first]['progress_m'],
             first_non_point_one_sim_s=following[0]['t'] if following else None)
    for limit in (.15, .35):
        found = next((row for row in following if row['drive_speed_mps'] > limit + 1e-6 and row['safe_speed_mps'] > limit + 1e-6), None)
        e['command_after_above_' + str(limit)] = None if found is None else {
            k: found[k] for k in ('t', 'progress_m', 'drive_speed_mps', 'safe_speed_mps', 'ground_speed_mps', 'mcu_state')}
    episodes.append(e)
    i += 1

guard_trials = [e for e in done if 'selector_guard_crawl_retry' in normal(e)]
current_diags = counts(k for e in curr for k in e['data']['current_rebase']['output'] if k.startswith('current_'))
accepted_curr = []
for e in curr:
    d = e['data']['current_rebase']['output']
    for name in ('current_corner_distance_budget', 'current_relative_body_retry', 'current_crop_component_retry'):
        if name in d:
            accepted_curr.append(dict(trace_seq=e['seq'], input_reference_s=e['data']['input']['reference_s'],
                                     diagnostic=name, value=d[name]))
write('current_diagnostics.json', accepted_curr)
write('compute_plan_wall_samples.json', [dict(generation=key(e)[0], sequence=key(e)[1],
    reference_sim_s=sm[key(e)]['data']['input']['reference_s'],
    compute_wall_ms=1000*(e['data']['completed_wall_s']-e['data']['began_wall_s']),
    submit_receive_wall_ms=1000*(e['data']['received_wall_s']-e['data']['submitted_wall_s'])) for e in done])

summary = dict(
    original_verdict={k: r.get(k) for k in ('passed', 'abort', 'harness_valid', 'run_valid', 'behavior_gates_passed', 'gates')},
    run_count=1, run_exit=int((B/'execution_v1/run.exit').read_text()), wall_elapsed_s=r['wall_elapsed_s'],
    progress=r['evaluation']['full_course_progress'], stop_request=read(B/'execution_v1/stop_requested.json'),
    new_guard_crawl=dict(diagnostic_count=len(guard_trials), accepted_count=sum(bool(normal(e)['selector_guard_crawl_retry'].get('accepted')) for e in guard_trials),
        parent_positive_publications=0 if not guard_trials else None, attributable_progress_m=None,
        continuous_point_one_duration_s=None, normal_acceleration_recovery=None,
        reason='new GUARD .1 candidate not entered; no attributable episode; do not attribute existing current corner .1 to it'),
    complete_trace=dict(kinds=counts(e['kind'] for e in t),
        submissions=len(sub), completions=len(done), unique_submissions=len(sm), unique_completions=len(dm),
        unmatched_submission_keys=sorted(set(sm)-set(dm)), unmatched_completion_keys=sorted(set(dm)-set(sm)),
        submitted_wall_disagreements=[list(k) for k in sm.keys() & dm.keys() if sm[k]['data']['submitted_wall_s'] != dm[k]['data']['submitted_wall_s']],
        generations=counts(e['data']['generation'] for e in sub),
        reset_reasons=counts(e['data'].get('reason') for e in by('reset')),
        seq_contiguous=[e['seq'] for e in t] == list(range(t[0]['seq'], t[-1]['seq'] + 1)),
        footer=t[-1], evaluator_header_records=len(by('runtime_evaluator_header')),
        note='submitted/completed are calculation evidence, not publication/application proof'),
    parent_route_publication=dict(total=len(pubs), retained_positive=sum(e['data']['retained'] and e['data']['speed_mps']>0 for e in pubs),
        first=pubs[0], last=pubs[-1], zero_commands_not_counted_by_route_publication=True),
    current_rechecks=dict(count=len(curr), diagnostics=current_diags,
        reasons=counts(e['data']['current_rebase']['output'].get('reason') for e in curr)),
    existing_point_one_episodes=episodes,
    stall=dict(input_window_reference_sim_s=[34.999, stalled[-1]['data']['input']['reference_s']],
        submissions=len(stalled), completions=len(st_done),
        measured_speed_signs=counts('negative' if e['data']['input']['speed_mps']<0 else 'positive' if e['data']['input']['speed_mps']>0 else 'zero' for e in stalled),
        plan_reasons=counts(normal(e).get('reason') for e in st_done), selector_reasons=counts(normal(e).get('selector_reason') for e in st_done),
        guard_lower_attempt_reasons=counts(str(a.get('target_mps'))+':'+str(a.get('reason')) for e in st_done for a in normal(e).get('selector_guard_speed_recheck',{}).get('attempts',[])),
        positive_plan_outputs=sum(e['data']['output'][0]>0 for e in st_done),
        current_rechecks=sum(e['data'].get('current_rebase') is not None for e in calls if e['data']['input']['reference_s']>=34.999),
        positive_parent_publications=sum(e['data']['at_s']>=34.999 and e['data']['speed_mps']>0 for e in pubs)),
    guard_mcu=dict(actuation_reasons=counts(a['reason'] for a in act), command_expired_samples=sum(a['reason']=='command_expired' for a in act),
        diagnostic=read(P/'actuation_diagnostics.json'), final_mcu=act[-1],
        safe_positive_trajectory_samples=sum(row['safe_speed_mps']>0 for row in traj),
        safety_reasons=counts(row['safety_reason'] for row in traj)),
    stop=dict(checks=r['stop_checks'], original_latch_count=r['actuation']['latched_before_stop_samples'],
        sequence=read(B/'stop_sequence.json'), final_output=r['actuation']['final_output']),
    contact_surface=dict(wall_collision_samples=r['wall_collision_samples'], contact_events=r['contact_evidence']['observed_events'],
        sampled_overlap_samples=r['evaluation']['gap']['sampled_overlap_samples'], road=r['evaluation']['road'],wheel_centers=r['evaluation']['wheel_centers']),
    cleanup=dict(process_exit_codes=r['process_exit_codes'], shutdown_audit=r['shutdown_audit'], remaining_test_pids=r['remaining_test_pids'], owned=read(B/'execution_v1/owned_processes_after.json')),
    integrity=read(B/'integrity.json'),
    scope='one early-stopped simulation; no driving repeat, physical evidence or whole-course success claim',
    analysis_only=True, common_source_changed=False)
write('summary.json', summary)
sealed = [p for p in P.iterdir() if p.is_file()] + [B/n for n in ('prepared.json','sources.zip','registered.json','integrity.json','runtime_audit.json','stop_sequence.json','root_execution_review.json')]
sealed += [B/'execution_v1'/n for n in ('run.exit','runner.log','stop_helper.exit','stop_helper.log','stop_requested.json','owned_processes_after.json','check_after_run.json')]
write('original_evidence_sha256.json', {str(p.relative_to(B)):sha(p) for p in sealed})
print(json.dumps({k:summary[k] for k in ('new_guard_crawl','current_rechecks','existing_point_one_episodes','stall')}, ensure_ascii=False, indent=2))
