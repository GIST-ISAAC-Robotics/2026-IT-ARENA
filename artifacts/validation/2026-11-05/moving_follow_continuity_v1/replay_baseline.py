"""11/5 기준 주행(moving_pass_baseline_v1/run1) 판단 입력 재생. 평가 전용, 정답 자세 미사용.

off: 기록 헤더 그대로 → 현재 코드가 기록 출력과 같은가(선택 OFF 동일성).
on : 헤더의 execution_config에 continuous_forward_pace=True만 더해 같은 입력을 재생(개방 루프 추정).
     이후 입력은 원래 폐루프 기록이므로 폐루프 결과의 예측이 아니다.
출력: replay_baseline.json.  WSL: environment.sh 경유 python3 -B replay_baseline.py
"""
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRACE = ROOT/'artifacts/validation/2026-11-05/moving_pass_baseline_v1/run1/follow_decisions.jsonl'
T_GREEN, T_END = 7.401, 21.401


def events(enable):
    with TRACE.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if enable and event['kind'] == 'header':
                event['data']['execution_config']['continuous_forward_pace'] = True
            yield event


def calls():
    with TRACE.open(encoding='utf-8') as stream:
        return [json.loads(l) for l in stream if '"kind":"call"' in l[:40]]


def summary(values):
    values = sorted(values)
    return dict(n=len(values), min=values[0], median=values[len(values)//2], max=values[-1]) if values else None


def main():
    off = replay_events(events(False))
    on = replay_events(events(True))
    recorded = {c['seq']: c['data'] for c in calls()}
    rows = []
    for m in on['mismatches']:
        data = recorded[m['seq']]
        ref = data['input']['reference_s']
        e, a = m['expected'], m['actual']
        fe_e, fe_a = e[2].get('follow_entry_space', {}), a[2].get('follow_entry_space', {})
        rows.append(dict(seq=m['seq'], reference_s=ref, in_window=T_GREEN <= ref <= T_END,
                         recorded_v=e[0], replay_v=a[0], steering_equal=e[1] == a[1],
                         recorded_reason=fe_e.get('reason'), replay_reason=fe_a.get('reason'),
                         observed_forward_pace_mps=fe_a.get('observed_forward_pace_mps'),
                         fixed_reserve_cap_mps=fe_a.get('fixed_reserve_cap_mps')))
    window = [d for d in recorded.values() if T_GREEN <= d['input']['reference_s'] <= T_END]
    changed = {r['seq'] for r in rows}
    rec_fallback = [s for s, d in recorded.items() if T_GREEN <= d['input']['reference_s'] <= T_END
                    and d['output'][2].get('follow_entry_space', {}).get('reason') == 'observed_entry_distance_reserve']
    replay_v = {s: d['output'][0] for s, d in recorded.items() if T_GREEN <= d['input']['reference_s'] <= T_END}
    for r in rows:
        if r['in_window']:
            replay_v[r['seq']] = r['replay_v']
    out = dict(scope='저장 입력 재생(개방 루프). 폐루프 성능 예측 아님.',
               trace=TRACE.relative_to(ROOT).as_posix(), trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
               bypass_handoff_sha256=hashlib.sha256((ROOT/'src/arena_autonomy/arena_autonomy/bypass_handoff.py').read_bytes()).hexdigest(),
               off=dict(calls=off['calls'], mismatch_count=len(off['mismatches'])),
               on=dict(calls=on['calls'], mismatch_count=len(on['mismatches']),
                       changed_calls_in_window=sum(r['in_window'] for r in rows),
                       changed_outside_window=[r['seq'] for r in rows if not r['in_window']],
                       all_steering_equal=all(r['steering_equal'] for r in rows),
                       all_changes_from_fallback=all(r['recorded_reason'] == 'observed_entry_distance_reserve' for r in rows),
                       replay_reasons=sorted({r['replay_reason'] for r in rows}),
                       never_faster_than_recorded_target=all(
                           r['replay_v'] <= recorded[r['seq']]['output'][2].get('follow_entry_space', {}).get('before_mps', float('inf')) + 1e-12
                           for r in rows),
                       observed_forward_pace_mps=summary([r['observed_forward_pace_mps'] for r in rows if r['observed_forward_pace_mps'] is not None])),
               recorded_fallback_calls_in_window=len(rec_fallback),
               fallback_calls_unchanged=len([s for s in rec_fallback if s not in changed]),
               command_below_0p10_in_window=dict(
                   recorded=sum(d['output'][0] < .10 for d in window),
                   replay_on=sum(v < .10 for v in replay_v.values())),
               command_in_window=dict(recorded=summary([d['output'][0] for d in window]),
                                      replay_on=summary(list(replay_v.values()))),
               changed=rows)
    (HERE/'replay_baseline.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: out[k] for k in ('off', 'on', 'recorded_fallback_calls_in_window', 'fallback_calls_unchanged',
                                          'command_below_0p10_in_window', 'command_in_window')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
