#!/usr/bin/env python3
"""재개 실행들의 STOP 요청 시각과 첫 MCU 고정 표본 시각 비교(읽기 전용). 판정을 바꾸지 않는다."""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
rows = {}
for run in sorted(p for p in BASE.glob('g1b_resume_v*_*_s*') if (p/'report.json').is_file()):
    report = json.loads((run/'report.json').read_text())
    trace = json.loads((run/'actuation_trace.json').read_text())
    latched = [r for r in trace if r['state'] in ('FAULT_LATCHED', 'ESTOP_LATCHED')]
    first = latched[0] if latched else None
    stop = report.get('stop_request_sim_s')
    rows[run.name] = {
        'passed': report.get('passed'), 'stop_request_sim_s': stop,
        'first_latched_now_us': first and first['now_us'], 'first_latched_reason': first and first['reason'],
        'float_seconds': first and first['now_us']*1e-6,
        'counted_before_stop_by_float': bool(first) and first['now_us']*1e-6 < stop,
        'same_millisecond_as_request': bool(first) and round(first['now_us']/1000) == round(stop*1000),
        'latched_before_stop_samples': report.get('actuation', {}).get('latched_before_stop_samples'),
        'reasons_of_latched_samples': sorted({r['reason'] for r in latched})}
out = Path(__file__).resolve().parent/'stop_latch_timing.json'
out.write_text(json.dumps({'scope': 'post-hoc comparison only; machine verdicts in report.json are unchanged', 'runs': rows},
                          indent=1), encoding='utf-8')
for name, row in rows.items():
    print(name, row)
