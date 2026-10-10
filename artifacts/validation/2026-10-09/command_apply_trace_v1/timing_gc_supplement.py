#!/usr/bin/env python3
"""보조 분석(등록 분석기 v1 이후 추가): 평가기 GC 정지와 노드 시계 도약·tick 지연의 중첩.

`analyze_evaluator_timing.py`의 범주는 직전 poll 간격만 본다. 정체 구간 안의 긴 poll 간격과 GC 정지를 함께 본다.
관측된 시간 중첩이며 인과 확정이 아니다. 인자: <출력 json> <실행 경로>...
"""
from bisect import bisect_left, bisect_right
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_evaluator_timing import gc_intervals, load_timing, read_jsonl  # noqa: E402


def analyze(run):
    run = ROOT/run
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    timing = load_timing(run, report)
    clock, after = timing['poll_node_clock_ns'], timing['poll_wall_after_spin_ns']
    rtf = (clock[-1]-clock[0])/(after[-1]-after[0])
    gcs = gc_intervals(read_jsonl(run/'gc_events.jsonl'))
    requests = read_jsonl(run/'requested_commands.jsonl')
    injected = (report.get('evaluation_fault_injection') or {}).get('target_scheduled_ns')
    long_gcs = []
    for g in gcs:
        if g['duration_ns'] < 20_000_000:
            continue
        i, j = bisect_right(after, g['begin_ns'])-1, bisect_left(after, g['end_ns'])
        if i < 0 or j >= len(after):
            continue
        # GC 직후 노드 시계가 처음 바뀐 poll까지의 도약.
        k = j
        while k+1 < len(clock) and clock[k] == clock[i]:
            k += 1
        crossed = [r for r in requests if clock[i] < r['scheduled_ns'] <= clock[k]]
        long_gcs.append({'generation': g['generation'], 'duration_ns': g['duration_ns'],
                         'sim_equivalent_ns_at_mean_rtf': round(g['duration_ns']*rtf),
                         'node_clock_before_ns': clock[i], 'node_clock_after_ns': clock[k],
                         'node_clock_jump_ns': clock[k]-clock[i],
                         'ticks_scheduled_inside': [{'index': r['index'], 'lateness_ns': r['observed_ns']-r['scheduled_ns'],
                                                     'injected_target': r['scheduled_ns'] == injected} for r in crossed]})
    late = []
    for r in requests:
        lateness = r['observed_ns']-r['scheduled_ns']
        if lateness < 5_000_000:
            continue
        j = bisect_left(clock, r['scheduled_ns'])
        k = j-1
        while k > 0 and clock[k-1] == clock[j-1]:
            k -= 1
        begin, end = after[max(k-1, 0)], after[j]
        gaps = [b-a for a, b in zip(after[max(k-1, 0):j], after[max(k-1, 0)+1:j+1])]
        overlap = sum(max(0, min(g['end_ns'], end)-max(g['begin_ns'], begin)) for g in gcs)
        late.append({'index': r['index'], 'lateness_ns': lateness, 'injected_target': r['scheduled_ns'] == injected,
                     'node_clock_jump_ns': clock[j]-clock[j-1], 'max_poll_gap_in_span_ns': max(gaps, default=None),
                     'gc_overlap_ns': overlap, 'gc_overlap_sim_equivalent_ns': round(overlap*rtf),
                     'gc_overlap_explains_half_of_jump': overlap*rtf >= .5*(clock[j]-clock[j-1])})
    return {'run': str(run.relative_to(ROOT)), 'mean_rtf': rtf,
            'gc_by_generation': {str(g): {'count': sum(1 for x in gcs if x['generation'] == g),
                                          'max_ns': max((x['duration_ns'] for x in gcs if x['generation'] == g), default=0)}
                                 for g in (0, 1, 2)},
            'gc_pauses_ge_20ms': long_gcs, 'late_ticks_ge_5ms': late}


if __name__ == '__main__':
    output, *runs = sys.argv[1:]
    result = {'runs': [analyze(r) for r in runs],
              'limits': ['GC 정지와 시계 도약의 시간 중첩만 보인다. OS 스케줄링·DDS·브리지 지연을 배제하지 않는다.',
                         'sim 환산은 실행 평균 RTF를 쓴 근사다.']}
    with open(output, 'x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    for r in result['runs']:
        print(r['run'], 'rtf=%.4f' % r['mean_rtf'], r['gc_by_generation'])
        for g in r['gc_pauses_ge_20ms']:
            print('  gc', g['generation'], '%.1f ms' % (g['duration_ns']/1e6), 'jump %.1f ms' % (g['node_clock_jump_ns']/1e6),
                  [(t['index'], t['lateness_ns']/1e6, t['injected_target']) for t in g['ticks_scheduled_inside']])
        for t in r['late_ticks_ge_5ms']:
            print('  late', t)
