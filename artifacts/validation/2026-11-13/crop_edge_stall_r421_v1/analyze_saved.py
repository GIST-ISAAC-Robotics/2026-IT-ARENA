"""r418 저장 필드의 계보/시각만 요약한다. 정책 import, 상태 재생, 주행은 하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
SOURCE = A.parent/'crop_edge_drive_r418_v1'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def identity(data):
    return (data['generation'], data['sequence'])


def content_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                   allow_nan=False).encode()).hexdigest()


def source_meta(data):
    scan = data['scan']; native = scan.get('measurements', {})
    return dict(reference_s=data['reference_s'], epoch=data['epoch'], speed_mps=data['speed_mps'],
        previous_steering_rad=data['previous_steering_rad'], timing_verified=data['timing_verified'],
        source_first_s=native.get('acquired_at_s', [None])[0], source_epoch=native.get('source_epoch'),
        source_frame_id=native.get('source_frame_id'))


def trace_row(index, data):
    autonomy = data.get('autonomy') or {}
    return dict(index=index, t=data['t'], progress_m=data['progress_m'],
        ground_speed_mps=data['ground_speed_mps'], drive_speed_mps=data['drive_speed_mps'],
        safe_speed_mps=data['safe_speed_mps'], safety_reason=data['safety_reason'],
        autonomy_reason=autonomy.get('reason'), autonomy_state=autonomy.get('state'),
        autonomy_sim_time_s=autonomy.get('sim_time_s'), mcu_state=data['mcu_state'])


def neighbors(rows, at):
    before = [r for r in rows if r['t'] <= at]
    after = [r for r in rows if r['t'] >= at]
    return dict(before=before[-1] if before else None, after=after[0] if after else None,
                scope='asynchronous samples bracketing recorded publication/reference; not command-ID identity')


def counts(calls):
    current = [c['current'] for c in calls if c['current'] is not None]
    return dict(raw_calls=len(calls), unique_plan_ids=len({tuple(c['plan_id']) for c in calls}),
        positive_plan_calls=sum(c['plan_speed_mps'] > 0 for c in calls),
        plan_reasons=dict(Counter(c['plan_reason'] for c in calls)),
        normal_reasons=dict(Counter(c['normal_reason'] for c in calls)),
        crop_diagnostic_calls=sum(c['crop_component'] is not None for c in calls),
        current_records=len(current), unique_current_ids=len({tuple(c['id']) for c in current}),
        positive_current_records=sum(c['speed_mps'] > 0 for c in current),
        current_reasons=dict(Counter(c['reason'] for c in current)),
        route_publications=len([c for c in calls if c.get('publication')]),
        positive_route_publications=sum(c.get('publication', {}).get('speed_mps', 0) > 0 for c in calls))


def budget_summary(value):
    if value is None:
        return None
    proposal = value.get('proposal', {})
    proof = value.get('proof', {})
    return dict(applied=value.get('applied'), reason=value.get('reason'),
        target_mps=value.get('target_mps'), original_target_mps=value.get('original_target_mps'),
        current_full_recheck=value.get('current_full_recheck'),
        allowed_distance_m=proposal.get('allowed_distance_m'),
        proposed_target_mps=proposal.get('proposed_target_mps'),
        proof_accepted=proof.get('accepted'), proof_reason=proof.get('reason'),
        proof_policy=proof.get('policy'), maximum_exterior_m=proof.get('exterior_max_m'))


def main():
    names = ['run1/trace.json', 'run1/follow_decisions.jsonl', 'runtime_audit.json', 'early_stop.json']
    hashes = {name: digest(SOURCE/name) for name in names}
    raw_trace = json.loads((SOURCE/'run1/trace.json').read_bytes())
    rows = [trace_row(index, row) for index, row in enumerate(raw_trace)]
    end = rows[-1]['t']; last_start = end-10.
    # 평가 progress는 양자화돼 있다. 처음 최대값에 도달한 표본이며 실제 정지 시각은 아니다.
    maximum = max(row['progress_m'] for row in rows)
    boundary = next(row for row in rows if row['progress_m'] == maximum)
    begin = boundary['t']-1.; finish = boundary['t']+1.
    plans, currents, calls, publications, events, conflicts = {}, {}, [], [], Counter(), []
    previous_event = None
    with (SOURCE/'run1/follow_decisions.jsonl').open(encoding='utf8') as stream:
        for line in stream:
            event = json.loads(line); kind = event['kind']; data = event['data']; seq = event['seq']
            events[kind] += 1
            if kind == 'call':
                compute = data['compute']; pid = identity(compute)
                value = content_digest([data['input'], data['output']])
                if pid in plans and plans[pid]['digest'] != value:
                    conflicts.append(dict(kind='plan', identity=pid, trace_seq=seq))
                plans.setdefault(pid, dict(digest=value, first_trace_seq=seq))
                output = data['output']; detail = output[2]; normal = detail.get('normal', detail)
                observation = normal.get('partial_observation') or {}
                call = dict(trace_seq=seq, plan_id=pid, plan_first_trace_seq=plans[pid]['first_trace_seq'],
                    plan_input=source_meta(data['input']), plan_speed_mps=output[0], plan_steering_rad=output[1],
                    plan_reason=detail.get('reason'), normal_reason=normal.get('reason'),
                    partial_allowed=observation.get('allowed'), partial_reason=observation.get('reason'),
                    crop_component=observation.get('crop_edge_component'), delivery=data['delivery'], current=None)
                rebase = data.get('current_rebase')
                if rebase:
                    current_compute = rebase['compute']; cid = identity(current_compute)
                    current_output = rebase['output']
                    value = content_digest([rebase['input'], current_output])
                    if cid in currents and currents[cid]['digest'] != value:
                        conflicts.append(dict(kind='current', identity=cid, trace_seq=seq))
                    currents.setdefault(cid, dict(digest=value, first_trace_seq=seq))
                    nested = current_compute['plan']
                    lineage = dict(plan_id_matches=identity(nested) == pid,
                        plan_input_matches=nested['input'] == data['input'],
                        plan_output_matches=nested['output'] == output,
                        current_input_matches=current_compute['input'] == rebase['input'])
                    if not all(lineage.values()):
                        conflicts.append(dict(kind='nested_lineage', trace_seq=seq, values=lineage))
                    proof = current_output.get('current_route_proof', {})
                    call['current'] = dict(id=cid, input=source_meta(rebase['input']),
                        speed_mps=current_output['speed_mps'], reason=current_output['reason'],
                        steering_rad=current_output['steering_rad'], reference_s=current_output['reference_s'],
                        original_reference_s=current_output.get('original_reference_s'),
                        same_raw_scan=data['input']['scan'] == rebase['input']['scan'], lineage=lineage,
                        original_speed_cap_mps=current_output.get('original_speed_cap_mps'),
                        path_age_s=current_output.get('path_age_s'), proof_accepted=proof.get('accepted'),
                        region=proof.get('region'), current_corner_distance_budget=budget_summary(current_output.get('current_corner_distance_budget')),
                        retries={key: current_output[key] for key in ('current_solo_gap_retry',
                            'current_relative_body_retry', 'current_solo_low_retry') if key in current_output})
                calls.append(call)
            elif kind == 'route_publication':
                publications.append(dict(trace_seq=seq, **data))
                # 발행 row에는 command/compute ID가 없다. 인접 event와 같은 reference의 대응만 기록한다.
                if previous_event and previous_event['kind'] == 'call':
                    call = calls[-1]; current = call['current']
                    if current is not None and current['reference_s'] == data['reference_s']:
                        call['publication'] = dict(trace_seq=seq, **data)
                        call['publication_link_basis'] = 'adjacent call event and exact current reference_s; publication has no compute ID'
            previous_event = event
    last = [c for c in calls if last_start <= c['plan_input']['reference_s'] <= end]
    near = [c for c in calls if begin <= c['plan_input']['reference_s'] <= finish]
    # 최초 current 거절·같은 scan의 마지막 대표·계산/부모 발행에 성공한 거리 후보를 구분한다.
    first_zero = next(c for c in calls if c['plan_input']['reference_s'] >= boundary['t']
        and c['plan_speed_mps'] > 0 and c['current'] and c['current']['speed_mps'] == 0)
    candidates = [c for c in calls if c['current'] and c['current']['current_corner_distance_budget']]
    representatives = [*candidates, first_zero, last[-1]]
    enriched = []
    for call in representatives:
        item = dict(call)
        when = call.get('publication', {}).get('at_s', call['current']['reference_s'])
        item['trace_neighbors'] = neighbors(rows, when)
        enriched.append(item)
    summary = dict(scope='saved fields only; no policy execution/state replay/source behavior proof',
        input_sha256=hashes, event_counts=dict(events), totals=counts(calls),
        identity_conflicts=conflicts, plan_duplicate_rows=len(calls)-len(plans),
        current_duplicate_rows=sum(c['current'] is not None for c in calls)-len(currents),
        linked_adjacent_publications=sum('publication' in c for c in calls),
        publications_total=len(publications),
        first_maximum_progress_sample=boundary,
        last_10_sim_seconds=dict(start_s=last_start, end_s=end, **counts(last),
            all_same_raw_scan=all(c['current']['same_raw_scan'] for c in last if c['current']),
            current_reference_offset_s=sorted({round(c['current']['reference_s']-c['plan_input']['reference_s'],9)
                for c in last if c['current']})),
        representatives=enriched,
        mcu_scope=dict(recorded_trace_fields=[key for key in raw_trace[0] if 'mcu' in key],
            running_target_speed_in_trace=False, final_stop=json.loads((SOURCE/'early_stop.json').read_bytes()),
            limitation='ARMED alone does not prove DRIVE receipt or actual target; early_stop final target is post-owned-STOP evidence'))
    selected = {c['trace_seq']: c for c in [*near, *last, *candidates]}
    detail = dict(boundary_window_s=[begin,finish], last_window_s=[last_start,end],
        calls=[selected[k] for k in sorted(selected)],
        boundary_trace=[r for r in rows if begin <= r['t'] <= finish])
    for name, value in [('summary.json', summary), ('selected_timeline.json', detail)]:
        with (A/name).open('x', encoding='utf8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    assert hashes == {name: digest(SOURCE/name) for name in names}
    print(json.dumps(dict(totals=summary['totals'], last10=summary['last_10_sim_seconds'],
        boundary=boundary, conflicts=conflicts, linked=summary['linked_adjacent_publications'],
        representative_seq=[c['trace_seq'] for c in enriched]), ensure_ascii=False))


if __name__ == '__main__':
    main()
