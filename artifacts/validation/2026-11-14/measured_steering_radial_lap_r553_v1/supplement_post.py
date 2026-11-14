"""저장된 r553 출력만 요약한다. 제어기 재생 및 원 결과 변경 없음."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
R = A / 'run1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interval_metrics(rows, predicate):
    total = longest = current = 0.0
    start = best_start = best_end = None
    for row, nxt in zip(rows, rows[1:]):
        dt = nxt['t'] - row['t']
        if predicate(row):
            if start is None:
                start = row['t']
            current += dt
            total += dt
            if current > longest:
                longest = current
                best_start, best_end = start, nxt['t']
        else:
            current = 0.0
            start = None
    duration = rows[-1]['t'] - rows[0]['t']
    return dict(total_sim_s=total, longest_contiguous_sim_s=longest,
                longest_range_s=[best_start, best_end], time_fraction=total / duration,
                matching_samples=sum(bool(predicate(r)) for r in rows), total_samples=len(rows))


def main():
    before = sha(R / 'report.json')
    report = read(R / 'report.json')
    evidence = read(A / 'evidence_collection.json')
    prepared = read(A / 'prepared.json')
    transition = read(A / 'stall_extract/transition.json')
    green, end = report['case_window']['green_sim_s'], report['case_window']['end_sim_s']
    rows = [json.loads(line) for line in (R / 'trajectory.jsonl').open(encoding='utf-8')]
    active = [r for r in rows if green <= r['t'] <= end]
    counts = collections.Counter()
    status_window = []
    zero_at = transition['first_zero_drive_trajectory']['t']
    nearest_zero_guard = None
    for line, text in enumerate((R / 'safety_status.jsonl').open(encoding='utf-8'), 1):
        row = json.loads(text)
        data = row['data']
        counts[data.get('detail', {}).get('mode', 'full_no_detail_metadata')] += 1
        ref = data.get('reference_s')
        if ref is not None and green <= ref <= end:
            status_window.append((ref, row['receipt_monotonic_s']))
        if ref is not None and (nearest_zero_guard is None or abs(ref-zero_at) < nearest_zero_guard[0]):
            nearest_zero_guard = (abs(ref-zero_at), dict(source_line=line, record=row))
    write(A / 'stall_extract/first_zero_drive_guard.json', nearest_zero_guard[1])
    pub_count = positive_pub_count = near_limit_count = over_limit_count = 0
    max_angle = 0.0
    limit = read(A / 'steering_runtime_receipt.json')['command_limit_rad']
    for text in (R / 'follow_decisions.jsonl').open(encoding='utf-8'):
        row = json.loads(text)
        if row['kind'] != 'route_publication':
            continue
        data = row['data']
        pub_count += 1
        positive_pub_count += data.get('speed_mps', 0) > 1e-6
        angle = abs(data['steering_rad'])
        max_angle = max(max_angle, angle)
        near_limit_count += abs(angle-limit) <= 1e-6
        over_limit_count += angle > limit + 1e-6
    source_map = report['source_sha256']
    actual_manifest = dict(scope='Actual run report source map; hashes of selected execution inputs, not whole repository',
                           inputs=source_map, source_snapshot_sha256=sha(R / 'source_snapshot.zip'))
    write(A / 'actual_execution_inputs.json', actual_manifest)
    required = prepared['frozen_inputs']
    outside_required = sorted(set(source_map) - set(required))
    metrics = dict(
        scope='Post-run saved evidence only; no replay; original verdict unchanged',
        original_report_sha256=before,
        actual_execution_inputs_sha256=sha(A / 'actual_execution_inputs.json'),
        actual_snapshot_count=len(source_map), actual_source_unchanged=report['source_unchanged'],
        actual_snapshot_missing=evidence['source_snapshot']['missing'],
        actual_snapshot_hash_mismatches=evidence['source_snapshot']['mismatches'],
        snapshot_entries_outside_required_freeze=outside_required,
        required_inputs=len(required), observed_inputs=len(prepared['observed_inputs']),
        raw_report_wall_elapsed_s=report['wall_elapsed_s'],
        active_receipt_rtf=(status_window[-1][0]-status_window[0][0])/(status_window[-1][1]-status_window[0][1]),
        active_sim_range=[status_window[0][0], status_window[-1][0]],
        active_receipt_wall_duration_s=status_window[-1][1]-status_window[0][1],
        rtf_scope='Delta guard reference / delta evaluator monotonic receipt within GREEN case window; includes delivery timing',
        trajectory_metric_scope='GREEN-window trajectory, left-held sample intervals; excludes intervals outside first/last recorded samples; no interpolation of missing rows',
        trajectory_metric_range_s=[active[0]['t'], active[-1]['t']],
        trajectory_metric_duration_s=active[-1]['t']-active[0]['t'],
        trajectory_max_interval_s=max(b['t']-a['t'] for a,b in zip(active, active[1:])),
        positive_definition='command > 1e-6 m/s',
        drive_positive=interval_metrics(active, lambda r:r['drive_speed_mps']>1e-6),
        safe_positive=interval_metrics(active, lambda r:r['safe_speed_mps']>1e-6),
        low_speed_definition='1e-6 < safe command <= 0.100001 m/s',
        safe_low=interval_metrics(active, lambda r:1e-6<r['safe_speed_mps']<=0.100001),
        safe_zero=interval_metrics(active, lambda r:abs(r['safe_speed_mps'])<=1e-6),
        original_stationary_metrics=report['evaluation']['case_metrics'],
        peak_ground_speed_mps=max(r['ground_speed_mps'] for r in active),
        prior_stall_milestones={str(p):next((dict(t=r['t'],progress_m=r['progress_m']) for r in active if r['progress_m']>p),None) for p in (19.56,25.81)},
        route_publications=pub_count, positive_route_publications=positive_pub_count,
        positive_route_publication_fraction=positive_pub_count/pub_count,
        maximum_abs_published_steering_rad=max_angle, near_limit_publications=near_limit_count,
        over_limit_publications=over_limit_count, steering_tolerance_rad=1e-6,
        status_rows=sum(counts.values()), status_detail_mode_counts=dict(counts),
        full_status_selected_and_recorded=prepared['status_detail']=='full' and set(counts)=={'full_no_detail_metadata'},
        raw_report_unchanged=sha(R/'report.json')==before,
    )
    assert metrics['raw_report_unchanged']
    write(A / 'run_supplement.json', metrics)
    print(json.dumps({k:metrics[k] for k in ['actual_snapshot_count','actual_execution_inputs_sha256','snapshot_entries_outside_required_freeze','active_receipt_rtf','drive_positive','safe_positive','safe_low','safe_zero','maximum_abs_published_steering_rad','over_limit_publications']},ensure_ascii=False))


if __name__ == '__main__':
    main()
