"""r441 원자료의 읽기 전용 집계. ROS/제어기 실행 및 원 판정 수정 없음."""
from __future__ import annotations

import collections
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import zipfile

B = Path(__file__).resolve().parent
P = B / 'run1'
O = B / 'analysis_v1'
O.mkdir(exist_ok=True)


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def lines(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8-sig').splitlines() if line]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, value):
    (O / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def stats(values):
    values = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    if not values:
        return {'n': 0}
    def q(frac):
        f = (len(values) - 1) * frac
        lo = int(f)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (f - lo)
    return dict(n=len(values), min=min(values), median=q(.5), p95=q(.95), max=max(values))


report = read(P / 'report.json')
trace = lines(P / 'follow_decisions.jsonl')
calls = [r for r in trace if r['kind'] == 'call']
pubs = [r for r in trace if r['kind'] == 'route_publication']
curr = [r for r in calls if r['data'].get('current_rebase')]
trajectory = lines(P / 'trajectory.jsonl')
statuses = lines(P / 'autonomy_status.jsonl')
actuation = read(P / 'actuation_trace.json')
observations = lines(B / 'execution_v1/observations.jsonl')
observed_wall = (datetime.fromisoformat(observations[-1]['at_utc']) - datetime.fromisoformat(observations[0]['at_utc'])).total_seconds()
observed_sim = observations[-1]['last']['t'] - observations[0]['last']['t']
normal = lambda r: r['data']['output'][2].get('normal', {})
stall = [r for r in calls if r['data']['input']['reference_s'] >= 34.12]

# Save raw rows as evidence; no reconstruction or re-evaluation of saved inputs.
selected = [r for r in trace if r['seq'] in (627, 628, 891, 892, 893, 895, 998)]
write('selected_original_rows.json', selected)

compute = []
for r in calls:
    d = r['data']
    for name, c in [('plan', d['compute']), ('current', d.get('current_rebase', {}).get('compute'))]:
        if c is None:
            continue
        compute.append(dict(seq=r['seq'], stage=name, input_reference_sim_s=d['input']['reference_s'],
                            computation_wall_ms=1000 * (c['completed_wall_s'] - c['began_wall_s']),
                            submit_to_receive_wall_ms=1000 * (c['received_wall_s'] - c['submitted_wall_s'])))
write('compute_wall_samples.json', compute)

diag_counts = collections.Counter()
for r in curr:
    diag_counts.update(k for k in r['data']['current_rebase']['output'] if k.startswith('current_'))
crop_count = sum('current_crop_component_retry' in r['data']['current_rebase']['output'] for r in curr)
crawl_count = sum(bool(r['data']['current_rebase']['output'].get('current_crop_component_retry', {}).get('crawl_retry')) for r in curr)

reasons = collections.Counter(a.get('reason') for a in actuation)
actuation_expired = [a for a in actuation if a.get('reason') == 'command_expired']
source_refs = []
with zipfile.ZipFile(B / 'sources.zip') as archive:
    for path, first, last in [
        ('src/arena_autonomy/arena_autonomy/pass_release_trial.py', 377, 428),
        ('src/arena_autonomy/arena_autonomy/g1_g2_node.py', 340, 348),
        ('src/arena_autonomy/arena_autonomy/race_runtime_policy/solo_relative.py', 13, 46),
    ]:
        raw = archive.read(path)
        outname = path.rsplit('/', 1)[-1] + '.excerpt.txt'
        text = '\n'.join(f'{i}: {line}' for i, line in enumerate(raw.decode('utf-8').splitlines(), 1)
                         if first <= i <= last)
        (O / outname).write_text(text + '\n', encoding='utf-8')
        source_refs.append(dict(archive_member=path, full_member_sha256=hashlib.sha256(raw).hexdigest(),
                                first_line=first, last_line=last, excerpt=outname))

attempt_stats = {}
for target in (.25, .15):
    attempts = [a for r in stall for a in normal(r).get('selector_guard_speed_recheck', {}).get('attempts', [])
                if a.get('target_mps') == target]
    attempt_stats[str(target)] = dict(count=len(attempts), accepted=sum(a['accepted'] for a in attempts),
        masked_tail_extent_m=stats(a['region']['masked_tail_extent_m'] for a in attempts),
        unobserved=stats(a['region']['unobserved'] for a in attempts),
        masked_front_samples=stats(a['region']['masked_front_samples'] for a in attempts))

costs = {}
for stage in ('plan', 'current'):
    cs = [c for c in compute if c['stage'] == stage]
    costs[stage] = dict(computation_wall_ms=stats(c['computation_wall_ms'] for c in cs),
                       submit_to_receive_wall_ms=stats(c['submit_to_receive_wall_ms'] for c in cs),
                       above_100_wall_ms=[c for c in cs if c['computation_wall_ms'] > 100])
costs['paired_plan_plus_current_wall_ms'] = stats(
    1000 * (r['data']['compute']['completed_wall_s'] - r['data']['compute']['began_wall_s'])
    + r['data']['current_rebase']['computation_wall_ms'] for r in curr)
costs['reference_to_publication_sim_ms'] = stats(1000 * (r['data']['at_s'] - r['data']['reference_s']) for r in pubs)
costs['scope'] = '이번 WSL ROS/Gazebo 실행의 wall timestamp. Windows 집계 실행시간/Jetson 처리시간이 아님. sim 차이와 별개.'

summary = dict(
    original_verdict={k: report[k] for k in ('passed', 'abort', 'harness_valid', 'run_valid', 'behavior_gates_passed', 'gates')},
    execution=dict(run_count=1, validation_returncode=read(B / 'integrity.json')['validation_returncode'],
                   wall_elapsed_s=report['wall_elapsed_s'], green_sim_s=report['green_observed_sim_s'],
                   case_window=report['case_window'], configured_rtf=.25,
                   actual_average_rtf_not_inferred_from_startup_and_cleanup_wall_time=True,
                   monitor_interval=dict(first_utc=observations[0]['at_utc'], last_utc=observations[-1]['at_utc'],
                       wall_seconds=observed_wall, sim_seconds=observed_sim, observed_ratio=observed_sim / observed_wall,
                       scope='Windows 관찰 시각과 마지막 저장 trajectory 표본 간 비율. Gazebo 내부 RTF 계측은 아님.')),
    progress=report['evaluation']['full_course_progress'],
    requested_stop=read(B / 'execution_v1/stop_requested.json'),
    new_crop_crawl=dict(current_crop_component_retry=crop_count, crawl_retry=crawl_count,
                       positive_parent_publication=0, body_progress_attributable=False,
                       limit='이번 trajectory에서 새 crop/crawl 분기 미호출. 성공이나 회귀 효과를 입증하지 않음.'),
    counts=dict(kinds=dict(collections.Counter(r['kind'] for r in trace)), current_diagnostics=dict(diag_counts),
                plan_reasons=dict(collections.Counter(normal(r).get('reason') for r in calls)),
                current_reasons=dict(collections.Counter(r['data']['current_rebase']['output'].get('reason') for r in curr))),
    stall=dict(first_persistent_plan_hold_seq=stall[0]['seq'], first_reference_sim_s=stall[0]['data']['input']['reference_s'],
               last_reference_sim_s=stall[-1]['data']['input']['reference_s'], call_count=len(stall),
               reasons=dict(collections.Counter(normal(r).get('reason') for r in stall)),
               selector_reasons=dict(collections.Counter(normal(r).get('selector_reason') for r in stall)),
               positive_plan_count=sum(r['data']['output'][0] > 0 for r in stall),
               current_recheck_count=sum(bool(r['data'].get('current_rebase')) for r in stall),
               last_positive_publication=pubs[-1], guard_lower_target_attempts=attempt_stats,
               first_normal=normal(stall[0]), last_normal=normal(stall[-1]),
               interpretation='기존 GUARD 곡선 후보 .35/.25/.15의 후측방 자기 mask 거절; parallel 직선 적합 실패로 plan 0. current crop 단계 미도달.'),
    existing_point_one_example=dict(seq=627, publication_seq=628,
        mechanism='current_relative_body_retry / bounded_relative_body_nonreturn_risk',
        current_proof=next(r for r in curr if r['seq'] == 627)['data']['current_rebase']['output']['current_relative_body_retry'],
        trajectory_nearby=[r for r in trajectory if 20.4 <= r['t'] <= 20.9]),
    command_expiry=dict(actuation_command_expired_samples=len(actuation_expired), actuation_reasons=dict(reasons),
                       actuation_diagnostics=read(P / 'actuation_diagnostics.json'),
                       no_deadline_diagnostic=report['stop_checks']['no_deadline_diagnostic']),
    computation=costs,
    stop=dict(checks=report['stop_checks'], settling=report['stop'],
              final_mcu=report['actuation']['final_mcu'], final_output=report['actuation']['final_output'],
              latched_before_validator_stop_samples=report['actuation']['latched_before_stop_samples'],
              qualification='owned helper STOP이 original validator STOP보다 먼저 도착하여 no_mcu_latch_before_stop=false. 원 실패판정 보존.'),
    observed_contact_surface=dict(wall_collision_samples=report['wall_collision_samples'],
        actor_contact_events=report['contact_evidence']['observed_events'],
        sampled_overlap_samples=report['evaluation']['gap']['sampled_overlap_samples'],
        road=report['evaluation']['road'], wheel_centers=report['evaluation']['wheel_centers'],
        limit='조기 종료한 관측구간의 샘플/이벤트 판정. 실물 안전이나 전체 80 sim초/완주 증거 아님.'),
    cleanup=dict(shutdown_audit=report['shutdown_audit'], process_exit_codes=report['process_exit_codes'],
                 remaining_test_pids=report['remaining_test_pids'], owned=read(B / 'execution_v1/owned_processes_after.json')),
    integrity=read(B / 'integrity.json'),
    runtime_audit=read(B / 'runtime_audit.json'),
    frozen_source_references=source_refs,
    raw_evidence_sha256={str(path.relative_to(B)): sha(path) for path in [
        P / 'report.json', P / 'follow_decisions.jsonl', P / 'trajectory.jsonl', P / 'actuation_trace.json',
        P / 'autonomy_status.jsonl', P / 'stop_trace.json', B / 'prepared.json', B / 'sources.zip',
        B / 'integrity.json', B / 'runtime_audit.json', B / 'execution_v1/owned_processes_after.json']},
    analysis_only=True, new_simulation_or_replay=False, shared_source_modified=False)
write('summary.json', summary)
print(json.dumps({k: summary[k] for k in ('new_crop_crawl', 'counts', 'command_expiry', 'computation')}, ensure_ascii=False, indent=2))
