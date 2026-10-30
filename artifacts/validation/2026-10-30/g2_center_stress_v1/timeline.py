"""센서 정체·역회전·경로 초기화의 순서를 저장 기록에서 분리한다. 제어에는 정답을 넣지 않는다."""
from collections import Counter
import bisect
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
RUN = BASE/'run1'


def main():
    calls = [e['data'] for line in (RUN/'g2_decisions.jsonl').open() if (e := json.loads(line))['kind'] == 'call']
    events = [json.loads(line) for line in (RUN/'braking_diagnostics.jsonl').open()]
    times = [r['sim_time_s'] for r in events]
    def near(t):
        i = min(len(events)-1, bisect.bisect_left(times, t))
        if i > 0 and abs(times[i-1]-t) < abs(times[i]-t):
            i -= 1
        return {k:events[i][k] for k in ('sim_time_s','left_speed_rad_s','right_speed_rad_s',
            'motor_speed_rad_s','reference_speed_mps','truth_longitudinal_mps')}
    selected = []
    for d in calls:
        inp, out = d['input'], d['output']
        if 11.18 <= inp['reference_s'] <= 11.42:
            detail = out[2]
            selected.append(dict(t=inp['reference_s'], measured=inp['speed_mps'], target=out[0],
                reason=detail['reason'], candidate_reasons=[c['reason'] for c in detail.get('candidates',[])],
                nearby_dynamics_evaluation_only=near(inp['reference_s'])))
    negatives = [d for d in calls if d['input']['speed_mps'] < 0]
    trace = json.loads((RUN/'trace.json').read_text())
    late = [r for r in trace if (r['elapsed_since_green_s'] or 0) > 8]
    result = dict(scope='saved-run causal ordering, physical truth only for evaluation',
        calls=len(calls), reason_counts=dict(Counter(d['output'][2]['reason'] for d in calls)),
        negative_feedback_calls=len(negatives), first_negative_s=negatives[0]['input']['reference_s'],
        last_positive_s=max(d['input']['reference_s'] for d in calls if d['output'][0] > 0),
        late_samples=len(late), late_positive_drive=sum(r['drive_speed_mps'] > 0 for r in late),
        late_positive_safe=sum(r['safe_speed_mps'] > 0 for r in late),
        late_max_body_speed_mps=max(r['ground_speed_mps'] for r in late),
        late_safety_reasons=dict(Counter(r['safety_reason'] for r in late)), events=selected,
        limits=['Single run; initial zero preceded wheel reversal but this alone is not a controlled mechanical causality test.',
                'Reverse motor feedback is not assumed to be noise: body truth is excluded from the candidate.'])
    (BASE/'run1_timeline.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'events'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
