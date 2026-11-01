"""저장 교통 입력에서 기준 명령을 재현하고, 선택 시 후보 판단을 같은 입력에 대조한다.

고정 입력 대조는 발행 전 명령 비교다. 실제 진행·접촉·궤적 결과가 아니다.
"""
import argparse
from collections import Counter
from dataclasses import fields, replace
from hashlib import sha256
import json
from pathlib import Path

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def run(folder, option):
    report = json.loads((folder / 'report.json').read_bytes())
    if not report['run_valid'] or not report['source_unchanged']:
        raise ValueError('유효한 실제 교통 입력 필요')
    window = report['case_window']
    green = window['green_sim_s']
    duration = window['duration_s']
    cuts = {'first_curve': (0., 6., 13., duration), 'stop_restart': (0., 4., 9., duration)}.get(
        report['case'], (0., 4., duration))
    results = [Counter() for _ in cuts[:-1]]
    digest = sha256()
    footer = None
    reproduced = 0
    last = None
    candidate = None
    with (folder / 'follow_decisions.jsonl').open('rb') as f:
        for seq, line in enumerate(f):
            digest.update(line)
            e = json.loads(line)
            d = e['data']
            if seq != e['seq'] or footer is not None:
                raise ValueError('판단 순번 또는 footer 오류')
            if e['kind'] == 'header':
                motion = MotionConfig(**d['motion_config'])
                config = FollowConfig(**d['follow_config'])
                params = HandoffExecutionConfig.from_mapping(d['execution_config'])
                original = BypassHandoffFollow(config, params, allow_simulation_trial=True)
                if option:
                    options = option.split(',')
                    if not set(options) <= {x.name for x in fields(params)}:
                        raise ValueError('후보 옵션이 아직 없음')
                    # 후보는 최근 전진 후보와 함께 쓰지 않는다. 그 실행의 입력만 대조한다.
                    candidate = BypassHandoffFollow(config, replace(params, recent_forward_reserve=False,
                                                                    **{name: True for name in options}),
                                                    allow_simulation_trial=True)
            elif e['kind'] == 'footer':
                footer = d
            elif e['kind'] == 'reset':
                original.reset()
                last = None
                if candidate is not None:
                    candidate.reset()
            elif e['kind'] == 'call':
                inp = d['input']
                t = inp['reference_s'] - green
                old, angle, q = d['output']
                v0, a0, base = replay_call(original, inp, motion)
                if max(abs(v0 - old), abs(a0 - angle)) > 1e-8 or base.get('handoff_state') != q.get('handoff_state'):
                    raise ValueError(f'기준 출력 재현 실패 seq={seq}')
                if t > duration:
                    continue
                reproduced += 1
                if candidate is None:
                    continue
                v, a, new = replay_call(candidate, inp, motion)
                if last and 0 <= t - last[0] <= .05:
                    for j, (start, end) in enumerate(zip(cuts, cuts[1:])):
                        dt = max(0., min(t, end) - max(start, last[0]))
                        results[j]['command_integral_difference_m'] += dt * last[1]
                        results[j]['covered_s'] += dt
                last = t, v - old
                for j, (start, end) in enumerate(zip(cuts, cuts[1:])):
                    if not start <= t < end:
                        continue
                    st = results[j]
                    st['calls'] += 1
                    st['higher'] += int(v > old + 1e-7)
                    st['lower'] += int(v < old - 1e-7)
                    st['new_zero'] += int(v <= .001 < old)
                    st['released_zero'] += int(old <= .001 < v)
                    st['state_changes'] += int(new.get('handoff_state') != q.get('handoff_state'))
                    st['steering_changes'] += int(abs(a - angle) > 1e-8)
                    st['max_raise_mps'] = max(st['max_raise_mps'], v - old)
                    st['max_cut_mps'] = max(st['max_cut_mps'], old - v)
    if (not footer or footer['written'] != seq or any(footer[k] for k in ('failed', 'dropped', 'capture_errors', 'pending'))
            or digest.hexdigest() != report['follow_decisions']['sha256']):
        raise ValueError('기록 전체성 확인 실패')
    return dict(run=str(folder), case=report['case'], trace_sha256=digest.hexdigest(),
                baseline_commands_reproduced=reproduced, cuts_s=cuts, phases=[dict(x) for x in results],
                scope='같은 측정·직전 실제 조향을 쓴 발행 전 비교. 실제 진행이나 접촉 결과가 아니다.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('out', type=Path)
    p.add_argument('runs', type=Path, nargs='+')
    p.add_argument('--option')
    a = p.parse_args()
    result = [run(folder, a.option) for folder in a.runs]
    with a.out.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    for r in result:
        print(r['run'], r['case'], r['baseline_commands_reproduced'], r['phases'])
