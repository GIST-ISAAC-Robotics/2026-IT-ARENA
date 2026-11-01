"""기존 목표·조향을 고정해 증속 명령의 여지만 계산한다. 차량 궤적 예측이 아니다."""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'race_acceleration_v1/run1'


def analyze():
    rows, speeds = [], {1.5: 0., 2.0: 0.}
    for line in (RUN/'follow_decisions.jsonl').open(encoding='utf-8'):
        event = json.loads(line)
        data = event['data']
        if event['kind'] == 'header':
            config = data['follow_config']
        elif event['kind'] == 'reset':
            speeds = {k: 0. for k in speeds}
        elif event['kind'] == 'call':
            target, steering, _ = data['output']
            inp = data['input']
            curvature = abs(math.tan(steering)/config['wheelbase_m'])
            if curvature > 1e-6:
                target = min(target, math.sqrt(config['lateral_acceleration_mps2']/curvature))
            speeds = {rate: min(target, before+rate*inp['dt_s']) for rate, before in speeds.items()}
            rows.append(dict(t=inp['reference_s'], target=target, dt=inp['dt_s'],
                             baseline=speeds[1.5], candidate=speeds[2.0]))
    by_time = {round(r['t'], 6): r for r in rows}
    published = {}
    for line in (RUN/'trajectory.jsonl').open(encoding='utf-8'):
        status = json.loads(line).get('autonomy') or {}
        t = status.get('sim_time_s')
        if t is not None and 'speed_command_mps' in status:
            published[round(t, 6)] = status['speed_command_mps']
    comparisons = [(t, abs(published[t]-by_time[t]['baseline'])) for t in published.keys() & by_time.keys()]
    if not comparisons or max(e for _, e in comparisons) > 1e-8:
        raise ValueError('기록된 발행 상태를 재현하지 못했으므로 증속 비교를 해석할 수 없다')
    summary = {}
    for key in ('target', 'baseline', 'candidate'):
        summary[key] = dict(max_mps=max(r[key] for r in rows),
            above_2p6_s=sum(r['dt'] for r in rows if r[key] >= 2.6),
            above_2_s=sum(r['dt'] for r in rows if r[key] >= 2.))
    return dict(source=str(RUN), rows=len(rows), matched_published_statuses=len(comparisons),
        max_published_error_mps=max(e for _, e in comparisons), summary=summary,
        changed_calls=sum(abs(r['baseline']-r['candidate']) > 1e-8 for r in rows),
        scope='기록된 경로·조향·센서 입력을 고정한 명령 증속 비교. 새 목표를 받고 달릴 실제 궤적·접지·보호 출력은 예측하지 않는다.')


if __name__ == '__main__':
    result = analyze()
    with (HERE/'rate_shadow.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
