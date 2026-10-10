#!/usr/bin/env python3
"""사후 진단 v2: 실행별 발행 위상과 마지막 속도 0 전환의 요청/safe/MCU 지연(시뮬레이션 시간).

v1(command_phase.py)은 마지막 상태 전환(속도·조향 포함)을 속도 0 전환으로 암묵 가정했다.
현재 프로필에서는 두 시각이 같음을 여기서 명시 검사하고, 속도 0 전환을 직접 계산한다.
"""
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PROFILE = json.loads((ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json').read_text())['samples_t_speed_steering']
TRANSITIONS = [b[0] for a, b in zip(PROFILE, PROFILE[1:]) if a[1:] != b[1:]]
LAST_POSITIVE = max(i for i, row in enumerate(PROFILE) if row[1] > 0)
SPEED_ZERO_S = PROFILE[LAST_POSITIVE+1][0]
RUNS = ['../zero_speed_v1/flat_passive_damping', '../zero_speed_v1/official_passive_damping', '../zero_speed_v1/flat_legacy',
        'flat_passive_0p5ms', 'flat_passive_0p25ms', 'official_passive_0p5ms', 'official_passive_0p25ms',
        'flat_legacy_0p25ms', 'flat_legacy_0p25ms_retry1', 'official_passive_1ms_rerun', 'official_passive_0p25ms_repeat']


def first_zero_after(rows, start, key):
    positive = [r for r in rows if r[0] >= start and r[1] > 0]
    return next((t for t, v in rows if t > positive[-1][0] and v == 0), None) if positive else None


result = {}
for name in RUNS:
    run = (BASE/name).resolve()
    report = json.loads((run/'report.json').read_text())
    start = report['command_start_sim_s']
    lines = lambda f: [json.loads(l) for l in (run/f).read_text().splitlines()]
    requests = [(r['sim_s'], r['speed_mps']) for r in lines('requested_commands.jsonl')]
    safe = [(r['sim_s'], r['speed_mps']) for r in lines('safe_commands.jsonl')]
    mcu = sorted({r['mcu']['now_us']: r['mcu']['target_speed_mps'] for r in lines('braking_diagnostics.jsonl') if r.get('mcu')}.items())
    mcu = [(t*1e-6, v) for t, v in mcu]
    times = [t-start for t, _ in requests if 0 <= t-start <= 3.5]
    lags = [round(min(t for t in times if t >= x)-x, 6) for x in TRANSITIONS if any(t >= x for t in times)]
    zero = {k: first_zero_after(v, start, k) for k, v in (('request', requests), ('safe', safe), ('mcu', mcu))}
    result[run.relative_to(ROOT).as_posix()] = {
        'first_publish_after_start_s': round(times[0], 6), 'transition_lags_s': lags, 'mean_lag_s': round(sum(lags)/len(lags), 6),
        'speed_zero_lag_after_profile_s': {k: round(v-start-SPEED_ZERO_S, 6) if v is not None else None for k, v in zero.items()}}
out = BASE/'command_phase_v2.json'
with out.open('x') as stream:
    json.dump({'transitions_s': TRANSITIONS, 'speed_zero_transition_s': SPEED_ZERO_S,
               'last_transition_is_speed_zero': TRANSITIONS[-1] == SPEED_ZERO_S, 'runs': result,
               'scope': '요청값은 각 발행 시각에서 프로필과 일치한다. 이 표는 발행/전달 위상만 보이며 위상이 출력 차이의 원인임을 증명하지 않는다.'},
              stream, indent=2)
print('last_transition_is_speed_zero', TRANSITIONS[-1] == SPEED_ZERO_S, SPEED_ZERO_S)
for k, v in result.items():
    print(k.split('/')[-1], v['first_publish_after_start_s'], v['mean_lag_s'], v['speed_zero_lag_after_profile_s'])
