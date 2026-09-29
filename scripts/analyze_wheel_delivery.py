#!/usr/bin/env python3
"""원시 계측을 읽는 엔코더 지연 분석. 제어 입력/보호 조건은 바꾸지 않는다."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path


def distribution(values):
    ordered = sorted(values)
    def percentile(fraction):
        return ordered[max(0, math.ceil(len(ordered)*fraction)-1)] if ordered else None
    return dict(count=len(ordered), p50=percentile(.5), p99=percentile(.99),
                maximum=ordered[-1] if ordered else None)


def callbacks_during_wait(rows, node, begin_ns, end_ns):
    """발행 뒤 지연 표본이 도착하기 전 다른 콜백이 계속 실행됐는지 대조한다."""
    selected = [r for r in rows if r['node'] == node
                and begin_ns <= r['begin_monotonic_ns'] < end_ns]
    result = {}
    for name in sorted({r['callback'] for r in selected}):
        items = [r for r in selected if r['callback'] == name]
        ticks = sorted([begin_ns, end_ns]+[r['begin_monotonic_ns'] for r in items])
        result[name] = dict(count=len(items), duration_ms=distribution([r['duration_ms'] for r in items]),
            maximum_start_gap_in_window_ms=max((b-a)/1e6 for a, b in zip(ticks, ticks[1:])))
    return result


def analyze(replay):
    report = json.loads((replay/'report.json').read_text(encoding='utf-8'))
    all_rows = []
    with (replay/'timing.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            all_rows.append(row)
    rows = [r for r in all_rows if r['callback'] == 'wheels']
    trace_path = replay/'wheel_publication.jsonl'
    publications = defaultdict(list)
    if trace_path.exists():
        for line in trace_path.read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            publications[row['source_ns']].append(row)
    results = {}
    for node in sorted({r['node'] for r in rows}):
        received = [r for r in rows if r['node'] == node]
        order = sorted(received, key=lambda r: r['begin_monotonic_ns'])
        ages = [r['source_age_ms'] for r in received]
        delivery = report.get('input_callback_delivery', {}).get(node+'/wheels', {})
        matched, ambiguous, absent = [], 0, 0
        for row in received:
            found = publications.get(row['source_ns'], [])
            if len(found) == 1:
                pub = found[0]
                matched.append(dict(source_ns=row['source_ns'],
                    publish_duration_ms=(pub['end_monotonic_ns']-pub['begin_monotonic_ns'])/1e6,
                    publish_begin_to_callback_ms=(row['begin_monotonic_ns']-pub['begin_monotonic_ns'])/1e6,
                    publish_end_to_callback_ms=(row['begin_monotonic_ns']-pub['end_monotonic_ns'])/1e6,
                    schedule_lateness_wall_ms=pub['schedule_lateness_wall_ms']))
            elif found:
                ambiguous += 1
            else:
                absent += 1
        worst = max(received, key=lambda r: r['source_age_ms'])
        worst_trace = publications.get(worst['source_ns'], [])
        wait_evidence = None
        if len(worst_trace) == 1:
            pub = worst_trace[0]
            wait_evidence = dict(source_ns=worst['source_ns'], publication=pub, callback=worst,
                callbacks_before_arrival=callbacks_during_wait(all_rows, node,
                    pub['begin_monotonic_ns'], worst['begin_monotonic_ns']),
                same_source_other_nodes=[r for r in rows if r['node'] != node and r['source_ns'] == worst['source_ns']])
        results[node] = dict(delivery=delivery,
            diagnostic_coverage=report.get('diagnostic_sequence_coverage', {}).get(node+'/wheels'),
            source_age_ms=distribution(ages),
            callback_duration_ms=distribution([r['duration_ms'] for r in received]),
            over_ms={str(limit): sum(v > limit for v in ages) for limit in (30, 100, 500)},
            source_time_reversals=sum(b['source_ns'] < a['source_ns'] for a, b in zip(order, order[1:])),
            matched_publications=len(matched), ambiguous_publications=ambiguous, absent_publications=absent,
            transport_and_dispatch_ms={key: distribution([r[key] for r in matched]) for key in
                ('publish_duration_ms', 'publish_begin_to_callback_ms', 'publish_end_to_callback_ms',
                 'schedule_lateness_wall_ms')},
            worst_source_age_samples=sorted(received, key=lambda r: r['source_age_ms'], reverse=True)[:5],
            worst_wait_evidence=wait_evidence)
    return dict(run=str(replay), wheel_publication_trace=trace_path.exists(), nodes=results,
        scope='Source age uses ROS time; publication/callback intervals use the same host monotonic clock. '
        'Publication end can occur after a receiver callback. DDS transport/queue and executor wait are not individually separated. '
        'No publication trace means that boundary is unobserved; it is not zero.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replays', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--recording', type=Path, help='최대 지연 표본과 원본 수신 시각의 읽기 전용 대조')
    args = parser.parse_args()
    results = [analyze(path) for path in args.replays]
    if args.recording is not None:
        wanted = {row['source_ns'] for result in results for node in result['nodes'].values()
                  for row in node['worst_source_age_samples']}
        original = defaultdict(list)
        with (args.recording/'receipt.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row['topic'] == '/wheel_states' and row['source_ns'] in wanted:
                    original[row['source_ns']].append(dict(source_ns=row['source_ns'],
                        receive_ros_ns=row['receive_ros_ns'],
                        recorded_source_age_ms=(row['receive_ros_ns']-row['source_ns'])/1e6))
        for result in results:
            stamps = {row['source_ns'] for node in result['nodes'].values()
                      for row in node['worst_source_age_samples']}
            result['original_receipt_samples'] = {str(stamp): original[stamp] for stamp in sorted(stamps)}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps([dict(run=r['run'], trace=r['wheel_publication_trace'], nodes={n: v['source_age_ms']
        for n, v in r['nodes'].items()}) for r in results]))


if __name__ == '__main__':
    main()
