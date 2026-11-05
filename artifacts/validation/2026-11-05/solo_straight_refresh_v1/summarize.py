"""재생 결과 집계(평가 전용). replay_*.jsonl.gz → summary.json.

정답 자세·진행 거리는 변경 위치를 사람이 읽도록 붙이는 평가값이며 제어 입력이 아니다.
부모 노드 모델(min(계획, sqrt(3/곡률), 직전+1.5·0.02))은 diagnosis_v1에서 실제 MCU 목표와 대조한 것과 같다(개루프).
"""
from collections import Counter
import gzip
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'solo_jr60p20_baseline_v1/run1'
ACCEL, DT, LAT, WB = 1.5, .02, 3.0, .135


def load(tag):
    with gzip.open(HERE/f'replay_{tag}.jsonl.gz', 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def publish(rows):
    last, out = 0., []
    for r in rows:
        v = r['v']
        k = abs(math.tan(r['steer'])/WB)
        if k > 1e-6:
            v = min(v, math.sqrt(LAT/k))
        v = min(v, last+ACCEL*DT)
        out.append(v)
        last = v
    return np.array(out)


def above(t, v, th):
    total = best = cur = 0.
    for a, b, va, vb in zip(t, t[1:], v, v[1:]):
        if va >= th and vb >= th:
            cur += b-a
            total += b-a
        else:
            best, cur = max(best, cur), 0.
    return dict(total_s=round(total, 3), longest_s=round(max(best, cur), 3))


def same_digest(a, b):
    return sum(x['digest'] == y['digest'] for x, y in zip(a, b)), len(a) == len(b)


def main():
    result = dict(scope=__doc__.strip())
    pairs = {}
    for name in ('solo_off', 'r13', 'mpb'):
        base, cand = load('base_'+name), load('cand_'+name)
        equal, aligned = same_digest(base, cand)
        pairs[name] = dict(calls=len(base), base_equals_cand=equal, aligned=aligned,
                           base_matches_recorded=sum(r['same_as_recorded'] for r in base),
                           cand_matches_recorded=sum(r['same_as_recorded'] for r in cand))
    result['off_identity'] = pairs
    off, on = load('cand_solo_off'), load('cand_solo_on')
    traj = [json.loads(x) for x in (RUN/'trajectory.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    tt = np.array([r['time'] for r in traj])
    ss = np.array([r['progress_m'] for r in traj])
    yaw_rate = np.array([r['dynamics']['truth_yaw_rate_rad_s'] for r in traj])
    ground = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in traj])
    changed = []
    for a, b in zip(off, on):
        assert a['seq'] == b['seq']
        if a['v'] != b['v'] or a['steer'] != b['steer'] or a['reason'] != b['reason']:
            s = float(np.interp(a['t'], tt, ss))
            g = float(np.interp(a['t'], tt, ground))
            w = float(np.interp(a['t'], tt, yaw_rate))
            changed.append(dict(t=round(a['t'], 3), progress_m=round(s, 2), off=round(a['v'], 4), on=round(b['v'], 4),
                                steer_delta=b['steer']-a['steer'], off_reason=a['reason'], on_reason=b['reason'],
                                budget=b['budget'], truth_path_curvature_per_m=round(abs(w)/max(g, .3), 4)))
    reasons = Counter(r['budget'] for r in on)
    up = [c for c in changed if c['on'] > c['off']]
    window = [c for c in changed if 42. <= c['progress_m'] <= 44.]
    zero_off = {r['seq'] for r in off if r['v'] <= .001}
    zero_on = {r['seq'] for r in on if r['v'] <= .001}
    t = np.array([r['t'] for r in off])
    p_off, p_on = publish(off), publish(on)
    lap = (t >= 8.077) & (t <= 8.077+32.949)
    cost_off = np.array([r['cost_s'] for r in off])*1e3
    cost_on = np.array([r['cost_s'] for r in on])*1e3
    segments = []
    for c in up:
        if segments and c['t']-segments[-1]['end_t'] <= .061:
            segments[-1].update(end_t=c['t'], end_progress_m=c['progress_m'], calls=segments[-1]['calls']+1,
                                max_on=max(segments[-1]['max_on'], c['on']), max_gain=max(segments[-1]['max_gain'], c['on']-c['off']))
        else:
            segments.append(dict(start_t=c['t'], end_t=c['t'], start_progress_m=c['progress_m'], end_progress_m=c['progress_m'],
                                 calls=1, max_on=c['on'], max_gain=c['on']-c['off']))
    result['solo_on'] = dict(
        calls=len(on), changed_calls=len(changed), speed_increased=len(up),
        speed_decreased=sum(c['on'] < c['off'] for c in changed),
        steering_changed=sum(c['steer_delta'] != 0. for c in changed),
        reason_changed=sum(c['off_reason'] != c['on_reason'] for c in changed),
        zero_calls_off=len(zero_off), zero_calls_on=len(zero_on), zero_set_identical=zero_off == zero_on,
        budget_reason_counts=dict(reasons.most_common()),
        increased_segments=segments,
        changed_in_42_44_m=len(window),
        max_truth_curvature_at_increase_per_m=max((c['truth_path_curvature_per_m'] for c in up), default=None),
        planner_peak_off_on=[round(max(r['v'] for r in off), 4), round(max(r['v'] for r in on), 4)],
        published_model=dict(
            peak_off_on=[round(float(p_off[lap].max()), 4), round(float(p_on[lap].max()), 4)],
            lap_command_distance_gain_m=round(float(np.sum(p_on[lap]-p_off[lap])*DT), 4),
            above_2p0_off=above(t[lap], p_off[lap], 2.0), above_2p0_on=above(t[lap], p_on[lap], 2.0),
            above_target95_off=above(t[lap], p_off[lap], .95*10/3.6), above_target95_on=above(t[lap], p_on[lap], .95*10/3.6),
            note='개루프 명령 모델. 실제 차체 속도·랩 시간이 아니다.'),
        replay_cost_ms=dict(off_p50=round(float(np.percentile(cost_off, 50)), 3), off_p95=round(float(np.percentile(cost_off, 95)), 3),
                            on_p50=round(float(np.percentile(cost_on, 50)), 3), on_p95=round(float(np.percentile(cost_on, 95)), 3),
                            on_minus_off_p95_per_call=round(float(np.percentile(cost_on-cost_off, 95)), 3),
                            note='WSL 재생 벽시계(nice 19, 다른 재생과 병렬). Jetson·ROS 지연 아님.'),
        changed=changed)
    (HERE/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result['solo_on'].items() if k not in ('changed',)}, ensure_ascii=False))
    print(json.dumps(result['off_identity'], ensure_ascii=False))


if __name__ == '__main__':
    main()
