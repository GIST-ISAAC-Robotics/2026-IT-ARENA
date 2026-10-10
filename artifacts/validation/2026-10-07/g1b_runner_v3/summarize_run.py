#!/usr/bin/env python3
"""G1-b 실행 요약(읽기 전용): 판정·핵심 지표·경계 선택 이력·소스 SHA."""
import hashlib, json, sys
from collections import Counter
from pathlib import Path

for name in sys.argv[1:]:
    run = Path('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-07')/name
    r = json.loads((run/'report.json').read_text())
    ev = r.get('evaluation', {})
    rows = [json.loads(l) for l in (run/'autonomy_status.jsonl').read_text().splitlines()]
    print('==', name, {k: r.get(k) for k in ('case', 'seed', 'mode', 'passed', 'run_valid', 'harness_valid', 'behavior_gates_passed',
                                             'shutdown_clean', 'source_unchanged', 'abort', 'wall_elapsed_s', 'remaining_test_pids')})
    print('   gates', {k: v for k, v in (r.get('gates') or {}).items() if v is not True} or 'all true', 'r0_meets_g1', r.get('r0_meets_g1_gates'))
    print('   gap', round(ev['gap']['min_sampled_gap_m'], 6), round(ev['gap']['min_swept_lower_bound_m'], 6),
          'progress', ev.get('progress', {}).get('at_window_end_m'), 'final_gap', ev.get('progress', {}).get('sampled_gap_at_window_end_m'))
    print('   case', ev.get('case_metrics'), 'road', ev.get('road', {}).get('outside_samples'), 'contact', ev['gap'].get('actor_contact_messages_with_ego'))
    print('   stop', r.get('stop'), 'diag', (r.get('actuation_diagnostics') or {}).get('samples'), (r.get('actuation_diagnostics') or {}).get('control_deadline_samples'))
    print('   reasons', dict(Counter(x.get('reason') for x in rows)), 'selections', dict(Counter(x.get('boundary_selection') for x in rows if x.get('boundary_selection'))))
    print('   unconfirmed_times', [round(x['sim_time_s'], 2) for x in rows if x.get('reason') == 'follow_boundary_unconfirmed'])
    print('   local_follow', r['source_sha256'].get('src/arena_autonomy/arena_autonomy/local_follow.py', '')[:12],
          'report', hashlib.sha256((run/'report.json').read_bytes()).hexdigest(), 'zip', hashlib.sha256((run/'source_snapshot.zip').read_bytes()).hexdigest())
