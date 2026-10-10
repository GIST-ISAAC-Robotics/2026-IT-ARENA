#!/usr/bin/env python3
"""격자 발행 평가기의 자연 발생 지연 위치를 좁히는 관측 분석(원인 확정 아님).

각 지연 tick에 대해 직전 poll과의 벽시계 간격·노드 시계 도약·노드 시계가 멈춰 있던 poll 수와 벽시계 길이,
같은 구간의 자체 /clock 구독 수신 공백, GC·느린 콜백 중첩을 계산하고 관측 범주를 붙인다.
- single_poll_wall_stall: 한 번의 poll 간격(벽시계) 동안 sim이 도약분 대부분을 진행
- node_clock_frozen_while_polling: 루프는 계속 돌았지만 노드 시계(TimeSource, depth 1)가 갱신되지 않음
  · own_clock_receipts_also_paused: 같은 구간 자체 /clock 구독 수신도 멈춤(상류 전달 공백과 양립)
  · own_clock_receipts_continued: 자체 구독은 받았는데 노드 시계만 정체(실행기 처리 순서와 양립)
범주는 관측된 패턴이며, OS 스케줄링·DDS·브리지 중 어느 것이 원인인지는 이 자료로 확정하지 않는다.
"""
import argparse
from array import array
from bisect import bisect_left, bisect_right
import json
from pathlib import Path

TIMING_ARRAYS = ('poll_wall_before_spin_ns', 'poll_wall_after_spin_ns', 'poll_node_clock_ns',
                 'clock_msg_sim_ns', 'clock_msg_wall_ns')


def read_jsonl(path):
    path = Path(path)
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()] if path.is_file() else []


def load_timing(run_dir, report):
    meta = report['evaluator_timing']
    raw = (Path(run_dir)/meta['path']).read_bytes()
    values, offset = {}, 0
    for name in TIMING_ARRAYS:
        count = meta['arrays_in_order'][name]
        data = array('q')
        data.frombytes(raw[offset:offset+8*count])
        offset += 8*count
        values[name] = data
    if offset != len(raw):
        raise ValueError('timing file length does not match report')
    return values


def overlapping(events, begin, end, key_begin, key_end):
    return [e for e in events if key_begin(e) <= end and key_end(e) >= begin]


def gc_intervals(gc_events):
    """start/stop 쌍을 벽시계 구간으로 묶는다."""
    out, pending = [], {}
    for event in gc_events:
        if event['phase'] == 'start':
            pending[event['generation']] = event['wall_ns']
        elif event['generation'] in pending:
            begin = pending.pop(event['generation'])
            out.append({'generation': event['generation'], 'begin_ns': begin, 'end_ns': event['wall_ns'],
                        'duration_ns': event['wall_ns']-begin, 'collected': event.get('collected')})
    return out


def classify_tick(stall, frozen, jump, rtf, receipt_gap, frozen_wall):
    if jump <= 0 or not rtf:
        return 'no_clock_jump'
    if stall*rtf >= .5*jump:
        return 'single_poll_wall_stall'
    if frozen >= 2 and frozen_wall*rtf >= .5*jump:
        paused = receipt_gap is not None and receipt_gap >= .5*frozen_wall
        return 'node_clock_frozen_while_polling:' + ('own_clock_receipts_also_paused' if paused
                                                       else 'own_clock_receipts_continued')
    return 'mixed_or_unresolved'


def analyze(run_dir, threshold_ns=5_000_000):
    run_dir = Path(run_dir)
    report = json.loads((run_dir/'report.json').read_text(encoding='utf-8'))
    timing = load_timing(run_dir, report)
    requests = read_jsonl(run_dir/'requested_commands.jsonl')
    gcs = gc_intervals(read_jsonl(run_dir/'gc_events.jsonl'))
    slow = read_jsonl(run_dir/'slow_callbacks.jsonl')
    clock, after, before = timing['poll_node_clock_ns'], timing['poll_wall_after_spin_ns'], timing['poll_wall_before_spin_ns']
    own_sim, own_wall = timing['clock_msg_sim_ns'], timing['clock_msg_wall_ns']
    if len(clock) < 2:
        raise ValueError('not enough poll samples')
    rtf = (clock[-1]-clock[0])/(after[-1]-after[0]) if after[-1] > after[0] else None
    jumps = [b-a for a, b in zip(clock, clock[1:])]
    histogram = {}
    for value in jumps:
        histogram[value] = histogram.get(value, 0)+1
    injected = (report.get('evaluation_fault_injection') or {}).get('target_scheduled_ns')
    ticks = []
    for row in requests:
        lateness = row['observed_ns']-row['scheduled_ns']
        if lateness < threshold_ns:
            continue
        j = bisect_left(clock, row['scheduled_ns'])
        if j <= 0 or j >= len(clock):
            ticks.append({'index': row['index'], 'lateness_ns': lateness, 'context': 'outside_recorded_polls'})
            continue
        stall = after[j]-after[j-1]
        jump = clock[j]-clock[j-1]
        k = j-1
        while k > 0 and clock[k-1] == clock[j-1]:
            k -= 1
        frozen_polls, frozen_wall = j-1-k, after[j-1]-after[k]
        lo, hi = bisect_right(own_wall, after[k]), bisect_right(own_wall, after[j])
        receipts = list(own_wall[lo:hi])
        bounds = [after[k], *receipts, after[j]]
        receipt_gap = max(b-a for a, b in zip(bounds, bounds[1:]))
        own_max_sim = max(own_sim[lo:hi], default=None)
        ticks.append({
            'index': row['index'], 'scheduled_ns': row['scheduled_ns'], 'lateness_ns': lateness,
            'injected_target': row['scheduled_ns'] == injected,
            'rel_profile_ms': (row['scheduled_ns']-report['command_schedule']['start_ns'])/1e6,
            'poll_wall_gap_ns': stall, 'spin_wall_ns': after[j]-before[j], 'node_clock_jump_ns': jump,
            'frozen_polls_before_jump': frozen_polls, 'frozen_wall_ns': frozen_wall,
            'own_clock_receipts_in_frozen_span': len(receipts), 'own_clock_max_receipt_gap_ns': receipt_gap,
            'own_clock_max_sim_in_span_ns': own_max_sim,
            'gc_overlap': overlapping(gcs, after[k], after[j], lambda e: e['begin_ns'], lambda e: e['end_ns']),
            'slow_callbacks_overlap': overlapping(slow, after[k], after[j], lambda e: e['wall_begin_ns'],
                                                  lambda e: e['wall_begin_ns']+e['duration_ns'])[:20],
            'category': classify_tick(stall, frozen_polls, jump, rtf, receipt_gap, frozen_wall)})
    gc_total = sum(g['duration_ns'] for g in gcs)
    return {'run': str(run_dir), 'threshold_ns': threshold_ns, 'polls': len(clock), 'real_time_factor_polls': rtf,
            'node_clock_jump_histogram_top': sorted(histogram.items(), key=lambda kv: -kv[1])[:12],
            'max_node_clock_jump_ns': max(jumps), 'max_poll_wall_gap_ns': max(b-a for a, b in zip(after, after[1:])),
            'own_clock_receipts': len(own_wall),
            'gc': {'collections': len(gcs), 'total_ns': gc_total, 'max_ns': max((g['duration_ns'] for g in gcs), default=0),
                   'by_generation': {str(g): sum(1 for x in gcs if x['generation'] == g) for g in (0, 1, 2)}},
            'slow_callbacks': len(slow), 'callback_stats': report['evaluator_timing'].get('callback_stats'),
            'late_ticks': ticks,
            'limits': ['범주는 관측 패턴이다. OS 스케줄링·DDS·브리지·Gazebo 발행 묶음 중 원인을 확정하지 않는다.',
                       '자체 /clock 수신 시각은 실행기가 콜백을 처리한 시각이며 DDS 도착 시각이 아니다.',
                       'RTF는 poll 첫/끝 표본의 평균이며 순간 RTF가 아니다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--threshold-ms', type=float, default=5.)
    args = parser.parse_args()
    result = {'runs': [analyze(path, round(args.threshold_ms*1e6)) for path in args.run]}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    for run in result['runs']:
        print(run['run'], 'rtf=%.4f' % (run['real_time_factor_polls'] or 0), 'max_jump_ms=%.2f' % (run['max_node_clock_jump_ns']/1e6),
              [(t['index'], t['lateness_ns']/1e6, t.get('category')) for t in run['late_ticks']])
