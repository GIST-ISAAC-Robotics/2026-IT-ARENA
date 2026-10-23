#!/usr/bin/env python3
"""재시작 진단 산출물 요약(읽기 전용). 판정이 아니라 원시 기록의 시간 순서 정리다.

사용: python3 -I analyze_restart_trace.py <validator_output 디렉터리> [출력 json]
"""
import json
from pathlib import Path
import sys

TOPICS = ('mcu', 'sink', 'applied', 'safe', 'safety', 'drive', 'autonomy')


def gaps(rows, topic, top=3):
    picked = [r for r in rows if r['topic'] == topic]
    result = []
    for before, after in zip(picked, picked[1:]):
        result.append(dict(sim_gap_s=round(after['sim_s'] - before['sim_s'], 6),
                           wall_gap_s=round(after['wall_s'] - before['wall_s'], 6),
                           from_sim_s=before['sim_s'], to_sim_s=after['sim_s'], to_seq=after['seq']))
    return dict(count=len(picked), largest_sim=sorted(result, key=lambda g: -g['sim_gap_s'])[:top],
                largest_wall=sorted(result, key=lambda g: -g['wall_gap_s'])[:top])


def source_lag(rows, topic, top=3):
    lags = [dict(lag_s=round(r['sim_s'] - r['source_sim_s'], 6), sim_s=r['sim_s'], source_sim_s=r['source_sim_s'],
                 seq=r['seq']) for r in rows if r['topic'] == topic and r['source_sim_s'] is not None]
    return sorted(lags, key=lambda x: -x['lag_s'])[:top]


def mcu_acceptance(rows):
    """ARMED 동안 accepted_seq가 바뀌지 않은 가장 긴 sim 구간."""
    best, start, last_seq = None, None, None
    for r in rows:
        if r['topic'] != 'mcu' or not isinstance(r['value'], dict):
            continue
        v = r['value']
        if v.get('accepted_seq') != last_seq or v.get('state') not in ('ARMED', 'ACTIVE'):
            if start is not None and (best is None or prev['sim_s'] - start['sim_s'] > best['span_s']):
                best = dict(span_s=round(prev['sim_s'] - start['sim_s'], 6), from_sim_s=start['sim_s'],
                            to_sim_s=prev['sim_s'], accepted_seq=last_seq, deadline_us=start['value'].get('deadline_us'))
            start = r if v.get('state') in ('ARMED', 'ACTIVE') else None
            last_seq = v.get('accepted_seq')
        prev = r
    if start is not None and (best is None or prev['sim_s'] - start['sim_s'] > best['span_s']):
        best = dict(span_s=round(prev['sim_s'] - start['sim_s'], 6), from_sim_s=start['sim_s'],
                    to_sim_s=prev['sim_s'], accepted_seq=last_seq, deadline_us=start['value'].get('deadline_us'))
    transitions = []
    state = None
    for r in rows:
        if r['topic'] == 'mcu' and isinstance(r['value'], dict):
            key = (r['value'].get('state'), r['value'].get('reason'), r['value'].get('generation'))
            if key != state:
                transitions.append(dict(sim_s=r['sim_s'], wall_s=r['wall_s'], seq=r['seq'], state=key[0],
                                        reason=key[1], generation=key[2], now_us=r['value'].get('now_us'),
                                        deadline_us=r['value'].get('deadline_us'),
                                        accepted_seq=r['value'].get('accepted_seq')))
                state = key
    return dict(longest_unchanged_accepted=best, transitions=transitions)


def ticks(trace):
    t = trace['ticks']
    pairs = list(zip(t, t[1:]))
    if not pairs:
        return {}
    wall = sorted(((b[0] - a[0], a[1], b[1]) for a, b in pairs), reverse=True)[:5]
    sim = sorted(((b[1] - a[1], a[1], b[1]) for a, b in pairs), reverse=True)[:3]
    return dict(count=len(t), sim_span_s=t[-1][1] - t[0][1], wall_span_s=t[-1][0] - t[0][0],
                largest_wall_gaps=[dict(wall_s=round(g, 6), from_sim_s=a, to_sim_s=b) for g, a, b in wall],
                largest_sim_steps=[dict(sim_s=round(g, 6), from_sim_s=a, to_sim_s=b) for g, a, b in sim])


def processes(trace, top=4):
    """주 스레드 run-queue 대기·실행 증가량이 가장 큰 표본 구간과 wchan 분포."""
    by_role, result = {}, {}
    for sample in trace['process_samples']:
        for p in sample['processes']:
            by_role.setdefault((p['role'], p['pid']), []).append((sample, p))
    for (role, pid), items in by_role.items():
        steps, wchan, states = [], {}, {}
        for (sa, a), (sb, b) in zip(items, items[1:]):
            if 'main_runqueue_wait_ns' in a and 'main_runqueue_wait_ns' in b:
                steps.append(dict(from_sim_s=sa['sim_s'], to_sim_s=sb['sim_s'],
                                  wall_s=round(sb['wall_s'] - sa['wall_s'], 6),
                                  wait_ms=round((b['main_runqueue_wait_ns'] - a['main_runqueue_wait_ns']) / 1e6, 3),
                                  run_ms=round((b['main_run_ns'] - a['main_run_ns']) / 1e6, 3),
                                  majflt=b['majflt'] - a['majflt'],
                                  cpu_ticks=(b['utime_ticks'] + b['stime_ticks']) - (a['utime_ticks'] + a['stime_ticks'])))
        for _, p in items:
            wchan[p.get('wchan')] = wchan.get(p.get('wchan'), 0) + 1
            states[p['state']] = states.get(p['state'], 0) + 1
        result[f'{role}:{pid}'] = dict(samples=len(items), wchan=wchan, states=states,
            largest_wait=sorted(steps, key=lambda s: -s['wait_ms'])[:top],
            largest_run=sorted(steps, key=lambda s: -s['run_ms'])[:top],
            majflt_total=sum(s['majflt'] for s in steps))
    pressure = []
    for a, b in zip(trace['process_samples'], trace['process_samples'][1:]):
        delta = {k: b['pressure'][k] - a['pressure'][k] for k in b['pressure'] if k in a['pressure']}
        if delta:
            pressure.append(dict(from_sim_s=a['sim_s'], to_sim_s=b['sim_s'], **delta))
    worst = {}
    for key in (pressure[0].keys() if pressure else ()):
        if key.endswith('_us'):
            worst[key] = sorted(pressure, key=lambda d: -d[key])[:2]
    return result, worst


def thread_activity(trace, lo, hi):
    """[lo, hi] sim 구간을 감싸는 스레드 표본 쌍에서 실행 증가량이 있는 스레드와 상태."""
    samples = [s for s in trace['process_samples'] if any('thread_rows' in p for p in s['processes'])]
    result = []
    for a, b in zip(samples, samples[1:]):
        if b['sim_s'] < lo or a['sim_s'] > hi:
            continue
        before = {(p['role'], t['tid']): t for p in a['processes'] for t in p.get('thread_rows', [])}
        rows = []
        for p in b['processes']:
            for t in p.get('thread_rows', []):
                old = before.get((p['role'], t['tid']))
                if old is None:
                    rows.append(dict(role=p['role'], tid=t['tid'], comm=t['comm'], state=t['state'], new=True))
                    continue
                run = (t['run_ns'] - old['run_ns']) / 1e6
                wait = (t['runqueue_wait_ns'] - old['runqueue_wait_ns']) / 1e6
                if run >= 1. or wait >= 1. or t['state'] not in ('S',):
                    rows.append(dict(role=p['role'], tid=t['tid'], comm=t['comm'], state=t['state'],
                                     run_ms=round(run, 2), wait_ms=round(wait, 2)))
        result.append(dict(from_sim_s=a['sim_s'], to_sim_s=b['sim_s'], wall_s=round(b['wall_s'] - a['wall_s'], 4),
                           active=sorted(rows, key=lambda r: -r.get('run_ms', 0))[:25]))
    return result


def probe_lines(output, log, prefix):
    path = output / log
    if not path.exists():
        return []
    return [json.loads(line[len(prefix):]) for line in path.read_text(errors='replace').splitlines()
            if line.startswith(prefix)]


def main():
    output = Path(sys.argv[1])
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    traces = json.loads((output / 'restart_traces.json').read_text(encoding='utf-8'))
    host_log = next((p['log'] for p in report['processes'] if p['role'] == 'host'), None)
    mcu_log = next((p['log'] for p in report['processes'] if p['role'] == 'mcu'), None)
    host_rows = probe_lines(output, host_log, 'HOST_DRIVE_DECISION=') if host_log else []
    summary = dict(passed=report.get('passed'), error=report.get('error'),
                   probe_log_summary=report.get('probe_log_summary'),
                   mcu_fault_contexts=report.get('mcu_fault_contexts'), traces=[])
    for trace in traces:
        rows = trace['rows']
        procs, pressure = processes(trace)
        window = (trace['start_sim_s'], trace['end_sim_s'])
        summary['traces'].append(dict(
            name=trace['name'], outcome=trace['outcome'], window_sim_s=window, rows_complete=trace['rows_complete'],
            ticks=ticks(trace), gaps={topic: gaps(rows, topic) for topic in TOPICS},
            source_lag={topic: source_lag(rows, topic) for topic in ('mcu', 'sink', 'safe', 'drive', 'autonomy')},
            mcu=mcu_acceptance(rows), processes=procs, pressure_worst=pressure,
            host_decisions=[r for r in host_rows if window[0] * 1e6 <= r.get('now_us', -1) <= window[1] * 1e6 + 1],
            slow_spins=trace.get('slow_spins', []),
            slow_publishes=[r for r in report.get('child_slow_publishes', []) + report.get('validator_slow_publishes', [])
                            if trace['start_wall_s'] <= r.get('start_wall_s', -1) <= trace['end_wall_s']]))
        stall = max(((b[0] - a[0], a[1]) for a, b in zip(trace['ticks'], trace['ticks'][1:])), default=(0, None))
        if stall[1] is not None:
            summary['traces'][-1]['largest_tick_gap'] = dict(wall_s=stall[0], sim_s=stall[1])
            summary['traces'][-1]['threads_around_largest_tick_gap'] = thread_activity(trace, stall[1] - .1, stall[1] + .05)
    text = json.dumps(summary, ensure_ascii=False, indent=1)
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(text, encoding='utf-8')
    else:
        print(text)


if __name__ == '__main__':
    main()
