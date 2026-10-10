#!/usr/bin/env python3
"""보조 분석: 같은 프로필 시작·같은 스폰 부류 쌍의 진단 분기 시작과 직전 명령 전환·단계 시각 차이.

추적이 있는 실행은 플러그인 적용 시각을, 없는 실행은 출력(속도)·MCU(속도·조향) 단계 시각을 쓴다.
관측된 선후 관계 정리이며 인과 확정이 아니다. 인자: <출력 json> <이름=경로>...
"""
from itertools import combinations
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_apply_trace import apply_events, divergence, spawn_from_diagnostics  # noqa: E402
from analyze_braking_ab import rows  # noqa: E402
from analyze_scheduled_phase import delivery  # noqa: E402
from braking_command_profile import load_profile  # noqa: E402
from scheduled_command import ProfileSampler  # noqa: E402


def stages(path):
    report = json.loads((path/'report.json').read_text(encoding='utf-8'))
    data = rows(path/'braking_diagnostics.jsonl')
    sampler = ProfileSampler(load_profile(ROOT/report['command_profile']['path']))
    start = report['command_schedule']['start_ns']
    d = delivery(path, report, data, sampler, 1_000_000)
    out = {'start_ns': start, 'data': data, 'spawn': spawn_from_diagnostics(data),
           'speed_output': d['speed']['stages_rel_start_ns'].get('output'),
           'speed_mcu': d['speed']['stages_rel_start_ns'].get('mcu'),
           'steering_mcu': d['steering']['stages_rel_start_ns'].get('mcu'), 'speed_apply': None, 'steering_apply': None}
    events = path/'plugin_command_events.jsonl'
    if events.is_file():
        ev = rows(events)
        for channel in ('speed', 'steering'):
            out[f'{channel}_apply'] = [e['sim_ns']-start for e in apply_events(ev, channel)
                                       if start <= e['sim_ns'] <= start+3_700_000_000]
    return out


def first_difference(a, b):
    if a is None or b is None or len(a) != len(b):
        return None
    diffs = [(x, y-x) for x, y in zip(a, b) if y != x]
    return {'first_at_rel_ns': diffs[0][0], 'diff_ns': diffs[0][1], 'count': len(diffs)} if diffs else {'first_at_rel_ns': None, 'count': 0}


def main():
    output, *items = sys.argv[1:]
    runs = {}
    for item in items:
        name, path = item.split('=', 1)
        runs[name] = stages(ROOT/path)
    pairs = []
    for a, b in combinations(runs, 2):
        x, y = runs[a], runs[b]
        if x['start_ns'] != y['start_ns'] or x['spawn'] != y['spawn']:
            continue
        div = divergence(x['data'], y['data'], x['start_ns'])
        onset = div['first_any_difference_rel_ns']
        item = {'a': a, 'b': b, 'spawn': x['spawn'], 'onset_rel_ns': onset,
                'exceeds_rel_ns': div['first_wheel_or_body_speed_exceeds_rel_ns'],
                'speed_output_first_difference': first_difference(x['speed_output'], y['speed_output']),
                'speed_mcu_first_difference': first_difference(x['speed_mcu'], y['speed_mcu']),
                'steering_mcu_first_difference': first_difference(x['steering_mcu'], y['steering_mcu']),
                'speed_apply_first_difference': first_difference(x['speed_apply'], y['speed_apply']),
                'steering_apply_first_difference': first_difference(x['steering_apply'], y['steering_apply'])}
        # 분기 직전(진단 한 표본 간격 21 ms 안)의 전환. a 실행의 가장 이른 관측 단계를 쓴다.
        near = []
        for channel in ('speed', 'steering'):
            times = x[f'{channel}_apply'] or x.get(f'{channel}_output') or x[f'{channel}_mcu']
            near += [{'channel': channel, 'rel_ns': t} for t in (times or []) if onset is not None and onset-21_000_000 <= t <= onset]
        item['transitions_within_21ms_before_onset_in_a'] = near
        pairs.append(item)
    result = {'runs': {k: {kk: v[kk] for kk in ('start_ns', 'spawn', 'speed_output', 'speed_apply', 'steering_apply')}
                       for k, v in runs.items()},
              'same_spawn_pairs': pairs,
              'scope': '보조 분석. 추적이 없는 실행의 적용 시각은 출력 단계에서 0~1 step 뒤라는 추적 실행의 관측으로만 추정된다.'}
    with open(output, 'x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    for p in pairs:
        print(p['a'], p['b'], p['spawn']['first_diag_sim_ns'], 'onset', p['onset_rel_ns'],
              'out1st', p['speed_output_first_difference'], 'apply', p['speed_apply_first_difference'],
              p['steering_apply_first_difference'], 'near', p['transitions_within_21ms_before_onset_in_a'])


if __name__ == '__main__':
    main()
