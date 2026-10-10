#!/usr/bin/env python3
"""사후 진단: 실행별 /drive 발행 위상과 프로필 전환 뒤 첫 발행 지연(시뮬레이션 시간)."""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PROFILE = json.loads((ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json').read_text())['samples_t_speed_steering']
TRANSITIONS = [b[0] for a, b in zip(PROFILE, PROFILE[1:]) if a[1:] != b[1:]]
RUNS = ['../zero_speed_v1/flat_passive_damping', '../zero_speed_v1/official_passive_damping', '../zero_speed_v1/flat_legacy',
        'flat_passive_0p5ms', 'flat_passive_0p25ms', 'official_passive_0p5ms', 'official_passive_0p25ms',
        'flat_legacy_0p25ms', 'flat_legacy_0p25ms_retry1', 'official_passive_1ms_rerun', 'official_passive_0p25ms_repeat']
result = {}
for name in RUNS:
    run = (BASE/name).resolve()
    report = json.loads((run/'report.json').read_text())
    start = report['command_start_sim_s']
    times = [json.loads(l)['sim_s']-start for l in (run/'requested_commands.jsonl').read_text().splitlines()]
    times = [t for t in times if 0 <= t <= 3.5]
    lags = [round(min(t for t in times if t >= x)-x, 6) for x in TRANSITIONS if any(t >= x for t in times)]
    result[run.relative_to(ROOT).as_posix()] = {'first_publish_after_start_s': round(times[0], 6), 'transition_lags_s': lags,
                                          'final_zero_lag_s': lags[-1], 'mean_lag_s': round(sum(lags)/len(lags), 6)}
with (BASE/'command_phase.json').open('x') as stream:
    json.dump({'transitions_s': TRANSITIONS, 'runs': result,
               'scope': '프로필 값은 각 발행 시각에서 일치한다. 이 표는 발행 위상 차이만 보이며 콜백/물리 step 시각 전체가 아니다.'},
              stream, indent=2)
for k, v in result.items():
    print(k.split('/')[-1], v['first_publish_after_start_s'], v['final_zero_lag_s'], v['mean_lag_s'])
