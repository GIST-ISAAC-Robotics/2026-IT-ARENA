"""r282 저장 시각/출력의 인과 구간 분해. 제어기·시한 변경 없음."""
import json
import math
import statistics
from collections import Counter
from pathlib import Path

A = Path(__file__).resolve().parent
D = A.parent / 'current_region_runtime_r282_v1' / 'two_actor_run1'


def lines(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def quantiles(values):
    values = sorted(values)
    return dict(n=len(values), minimum=values[0], median=statistics.median(values),
                p95=values[min(len(values)-1, math.ceil(len(values)*.95)-1)], maximum=values[-1]) if values else None


calls = []
for e in lines(D/'run1/follow_decisions.jsonl'):
    if e['kind'] != 'call':
        continue
    c = e['data']
    if c['delivery']['kind'] != 'candidate' or not c.get('current_rebase'):
        continue
    r = c['current_rebase']
    if 15 <= r['compute']['result_available_sim_s'] < 60:
        calls.append(dict(ready_s=r['compute']['result_available_sim_s'],
                          reference_s=r['output']['reference_s'], speed=r['output']['speed_mps']))

# 이 구간은 모든 현재 결과가 양수이며 새 결과가 없다면 guard는 원 시각 +200ms에 만료한다.
assert calls and all(c['speed'] > 0 for c in calls)
gaps = [max(0., b['ready_s']-a['reference_s']-.2) for a,b in zip(calls,calls[1:])]
safety = [r for r in lines(D/'full_safety.jsonl') if 15 <= r['reference_s'] < 60]
mcu = [r for r in json.loads((D/'run1/actuation_trace.json').read_bytes()) if 15e6 <= r['now_us'] < 60e6]
poses = [r for r in lines(D/'run1/ego_poses.jsonl') if 15 <= r[0] < 60]

# 가상 수신기 목표의 양수/0 구간 길이. Gazebo 내부 토크 기록으로 오인하지 않는다.
runs = []
start = mcu[0]['now_us']/1e6
positive = mcu[0]['target_speed_mps'] > 0
for a,b in zip(mcu,mcu[1:]):
    new = b['target_speed_mps'] > 0
    if new != positive:
        end = b['now_us']/1e6
        runs.append(dict(positive=positive, duration_s=end-start, start_s=start, end_s=end))
        start,positive = end,new

distance = sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(poses,poses[1:]))
result = dict(
    interval_s=[15,60], current_results=len(calls), all_current_results_positive=True,
    ready_intervals_s=quantiles([b['ready_s']-a['ready_s'] for a,b in zip(calls,calls[1:])]),
    ready_age_s=quantiles([c['ready_s']-c['reference_s'] for c in calls]),
    predicted_expiry_gaps_s=quantiles(gaps), predicted_gap_total_s=sum(gaps),
    guard_reasons=dict(Counter(r['reason'] for r in safety)),
    guard_stale_samples=sum(r.get('command_stale') is True for r in safety),
    mcu_reasons=dict(Counter(r['reason'] for r in mcu)),
    mcu_positive_spans_s=quantiles([r['duration_s'] for r in runs if r['positive']]),
    mcu_zero_spans_s=quantiles([r['duration_s'] for r in runs if not r['positive']]),
    body_average_speed_mps=distance/(poses[-1][0]-poses[0][0]),
    motor_average_speed_mps=statistics.mean(r['mean_wheel_mps'] for r in mcu),
    max_ramp_per_positive_span_mps=quantiles([3*r['duration_s'] for r in runs if r['positive']]),
    limits=['ready time is the current-result availability, not a byte-level /drive reception timestamp',
            'ramp bound is 3 m/s2 times MCU positive duration; no Gazebo torque/PI state was recorded',
            'later contact and region rejection are separate failures; this interval has no contact',
            'evaluation body pose is not supplied to the controller'])
(A/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result))
