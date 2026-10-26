"""저장 주행의 간격·횡이동·실제 판단 시간축. 제어 입력/판정을 바꾸지 않음."""
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import validate_g1_traffic as runner
from arena_autonomy.follow_trace import thaw


def main(mode):
    base = OUT.parent/'e3_onroad_v1'/mode
    files = [base/name for name in ('report.json', 'ego_poses.jsonl', 'additional_actors.json',
                                  'follow_decisions.jsonl', 'autonomy_status.jsonl')]
    report = json.loads(files[0].read_text())
    ego = [json.loads(line) for line in files[1].read_text().splitlines()]
    extra = json.loads(files[2].read_text())
    # 기록 형식이 다르면 실패로 드러내며 임의로 다른 형식을 고르지 않는다.
    actor = extra['poses']['eval_e3_side']
    line, _ = runner.load_centerline()
    times = [r[0] for r in ego]
    actor_times = [r[0] for r in actor]
    calls = []
    for raw in files[3].open():
        event = json.loads(raw)
        if event['kind'] != 'call':
            continue
        call = thaw(event['data'])
        inp, output = call['input'], call['output']
        t = inp['reference_s']
        if t is None:
            continue
        ei, ai = bisect_right(times, t)-1, bisect_right(actor_times, t)-1
        if ei < 0 or ai < 0 or t-times[ei] > .05 or t-actor_times[ai] > .05:
            continue
        _, x, y, yaw = ego[ei]
        _, ax, ay, ayaw = actor[ai]
        along, _ = line.nearest_s(x, y)
        cx, cy, heading = line.pose(along)
        c, s = math.cos(yaw), math.sin(yaw)
        detail = output[2]
        gap = runner.rectangle(x,y,yaw,.20,.15).distance(runner.rectangle(ax,ay,ayaw,.20,.15))
        calls.append({'t': t, 'ego_lateral_m': -(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading),
            'yaw_from_road_rad': math.atan2(math.sin(yaw-heading), math.cos(yaw-heading)),
            'side_longitudinal_m': c*(ax-x)+s*(ay-y), 'side_lateral_m': -s*(ax-x)+c*(ay-y),
            'sampled_gap_m': gap, 'measured_motor_speed_mps': inp['speed_mps'],
            'decision_speed_mps': output[0], 'decision_steering_rad': output[1],
            **{k: detail.get(k) for k in ('reason', 'behavior', 'path_wall', 'boundary_selection',
                'wall_fallback', 'boundary_from_memory', 'bilateral_confirmed', 'corridor_selected',
                'swept_clearance_m', 'target_speed_before_hold_mps')}})
    first, last = (report['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    window = [r for r in calls if first <= r['t'] <= last]
    # 기존 0.15 m 판정은 그대로 두고 진단 표만 만든다.
    minimum = min(window, key=lambda r:r['sampled_gap_m'])
    close = [r for r in window if abs(r['side_longitudinal_m']) < .35]
    positive_close = [r for r in close if r['decision_speed_mps'] > 0]
    first_hold = next((r for r in window if r['t'] > first+.3 and r['behavior']=='HOLD'), None)
    timeline = [min(window,key=lambda r:abs(r['t']-t)) for t in
                (first, first+.4, first+.8, first+1.2, first+1.6, first+2., first+2.4, first+3., last)]
    result = {'mode': mode, 'scope': 'recorded decision and sampled geometry, not counterfactual blame or new verdict',
        'original_passed': report['passed'], 'original_run_valid': report['run_valid'],
        'failed_main_behavior': [k for k,v in report['evaluation']['gates'].items() if not v],
        'failed_side_behavior': [k for k,v in report['additional_actors']['actors']['eval_e3_side']['gates'].items() if not v],
        'decision_rows': len(window), 'reason_counts':dict(Counter(r['reason'] for r in window)),
        'minimum_at_decision': minimum, 'first_hold_after_initial_0p3s': first_hold,
        'alongside_0p35m_positive_decisions': len(positive_close),
        'last_positive_alongside': positive_close[-1] if positive_close else None,
        'timeline':timeline,
        'sources':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
    with (OUT/f'{mode}.json').open('x') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    if len(sys.argv)!=2 or sys.argv[1] not in ('g1','r0plus'):
        raise SystemExit('expected g1 or r0plus')
    main(sys.argv[1])
