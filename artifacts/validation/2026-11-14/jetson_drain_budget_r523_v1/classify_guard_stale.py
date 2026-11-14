"""저장 로그만으로 원 명령 만료와 관찰된 0 명령 구간을 분리한다."""
import gzip
import json
from collections import Counter
from pathlib import Path

from analyze import quantiles

A = Path(__file__).resolve().parent
START, END = 7.62, 27.52


def classify(root):
    def rows(name):
        with gzip.open(root / (name + '.jsonl.gz'), 'rt', encoding='utf8') as stream:
            return [json.loads(line) for line in stream]

    commands = rows('commands')
    guard = [row['value'] for row in rows('guard_status')]
    by_reference = {round(row['reference_s'] * 1e9): row for row in guard}
    controls = json.loads((root / 'guard_controls.json').read_bytes())
    controls_by_reference = {round(row['control_reference_s'] * 1e9): row for row in controls}
    stale = []
    for status in guard:
        ref = status['reference_s']
        if not (START <= ref < END and status.get('command_stale') is True):
            continue
        control = controls_by_reference[round(ref * 1e9)]
        source = control['command_source_s']
        stale.append(dict(reference_s=ref, original_command_source_s=source,
                          original_command_age_ms=(ref-source)*1000))

    safe = [r for r in commands if r['topic'] == '/drive/safe' and START <= r['received_s'] < END]
    first_positive = next(r for r in safe if r['speed'] > 0)
    after_start = [r for r in safe if r['received_s'] >= first_positive['received_s']]
    intervals, pending = [], []

    def finish(next_positive):
        end = END if next_positive is None else next_positive['received_s']
        reasons = Counter()
        for r in pending:
            status = by_reference.get(round(r['source_s'] * 1e9))
            reasons['unmatched' if status is None else status['reason']] += 1
        return dict(start_received_s=pending[0]['received_s'], end_received_s=end,
                    active_duration_ms=(end-pending[0]['received_s'])*1000,
                    zero_samples=len(pending), reasons=dict(reasons),
                    first_guard_header_s=pending[0]['source_s'],
                    next_positive_guard_header_s=None if next_positive is None else next_positive['source_s'],
                    guard_header_duration_ms=None if next_positive is None else
                    (next_positive['source_s']-pending[0]['source_s'])*1000,
                    right_censored=next_positive is None)

    for row in after_start:
        if row['speed'] == 0:
            pending.append(row)
        elif pending:
            intervals.append(finish(row))
            pending = []
    if pending:
        intervals.append(finish(None))

    closed = [r for r in intervals if not r['right_censored']]
    all_zero_ms = sum(r['active_duration_ms'] for r in intervals)
    elapsed_ms = (END-first_positive['received_s'])*1000
    drive = [r for r in commands if r['topic'] == '/drive' and
             first_positive['received_s'] <= r['received_s'] < END]
    positive = [r for r in drive if r['speed'] > 0]
    expiry_windows = []
    for prev, nxt in zip(drive, drive[1:]):
        if prev['speed'] <= 0:
            continue
        # Observer receipt is a proxy, not the independent guard command callback.
        duration = max(0, nxt['received_s'] - max(prev['received_s'], prev['source_s'] + .2))
        if duration:
            expiry_windows.append(dict(previous_source_s=prev['source_s'],
                previous_received_s=prev['received_s'], next_received_s=nxt['received_s'],
                expired_window_ms=duration*1000))

    return dict(active_interval_s=[START, END], first_positive_safe_received_s=first_positive['received_s'],
        active_command_stale_count=len(stale),
        stale_original_command_age_ms=quantiles(r['original_command_age_ms'] for r in stale),
        stale_original_command_age_min_ms=min(r['original_command_age_ms'] for r in stale),
        active_guard_reasons=dict(Counter(r['reason'] for r in guard if START <= r['reference_s'] < END)),
        after_first_positive_safe_zero_samples=sum(r['speed'] == 0 for r in after_start),
        closed_zero_intervals=len(closed), closed_zero_sample_histogram=dict(Counter(r['zero_samples'] for r in closed)),
        closed_zero_received_duration_ms=quantiles(r['active_duration_ms'] for r in closed),
        closed_zero_guard_header_duration_ms=quantiles(r['guard_header_duration_ms'] for r in closed),
        closed_zero_total_ms=sum(r['active_duration_ms'] for r in closed),
        right_censored_zero_intervals=[r for r in intervals if r['right_censored']],
        after_first_positive_active_ms=elapsed_ms, observed_zero_target_total_ms=all_zero_ms,
        observed_zero_target_fraction=all_zero_ms/elapsed_ms,
        positive_drive_source_age_at_observer_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in positive),
        positive_drive_received_interval_ms=quantiles((b['received_s']-a['received_s'])*1000 for a,b in zip(positive,positive[1:])),
        observer_predicted_expiry_windows=dict(count=len(expiry_windows),
            duration_ms=quantiles(r['expired_window_ms'] for r in expiry_windows),
            total_ms=sum(r['expired_window_ms'] for r in expiry_windows)),
        boundaries=[
            'Guard original-command age uses control.command_source_s, never refreshed /drive/safe header.',
            'Zero intervals are ROS observer receipt intervals; guard-header intervals are listed separately.',
            'Initial departure and time after the last source-covered cut at 27.52s are excluded.',
            'The final zero interval is right censored; only its active portion is counted.',
            'Zero-target time is not measured physical stopping time, distance, or speed loss.',
            'Observer expiry-window prediction does not measure actual guard callback delivery.',
        ], zero_intervals=intervals, stale_age_evidence=stale)


if __name__ == '__main__':
    result = dict(stages={
        'r522_controller2ms': classify(A.parent / 'jetson_traceoff_continuity_r522_v1/remote_result/result'),
        'r523_controller6ms': classify(A / 'remote_result/result'),
    })
    (A / 'guard_stale_classification.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    for label, row in result['stages'].items():
        print(label, json.dumps({k: v for k, v in row.items()
              if k not in ('zero_intervals', 'stale_age_evidence', 'boundaries')}, ensure_ascii=False))
