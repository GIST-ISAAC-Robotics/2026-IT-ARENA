"""단독 완주 입력에서 시간 간격만 비교한다. 실제 새 주행 결과가 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent
TRACE = HERE.parent/'solo_route_v1/run1/follow_decisions.jsonl'


def run(headway):
    counts = Counter(); output_counts = Counter(); changes = calls = 0
    zero_to_positive = positive_to_zero = 0
    sum_target = sum_old = 0.
    maximum = 0.
    rows = []
    for line in TRACE.open(encoding='utf-8'):
        event = json.loads(line); data = event['data']
        if event['kind'] == 'header':
            config = FollowConfig(**data['follow_config'])
            if not config.solo_route_only or config.headway_s != .8:
                raise ValueError('단독 0.8초 기준 입력이 아님')
            follow = LocalFollow(replace(config, headway_s=headway))
            motion = MotionConfig(**data['motion_config'])
        elif event['kind'] == 'reset':
            follow.reset()
        elif event['kind'] == 'call':
            inp = data['input']; old_v, old_s, old_detail = data['output']
            v, s, detail = replay_call(follow, inp, motion)
            calls += 1; maximum = max(maximum, v)
            dt = inp['dt_s']; sum_target += v*dt; sum_old += old_v*dt
            output_counts[detail.get('reason')] += 1
            changed = abs(v-old_v) > 1e-8 or abs(s-old_s) > 1e-8
            changes += changed
            zero_to_positive += old_v == 0 and v > 0
            positive_to_zero += old_v > 0 and v == 0
            if changed:
                counts[(old_detail.get('reason'), detail.get('reason'))] += 1
                rows.append(dict(t=inp['reference_s'], before=[old_v,old_s], after=[v,s],
                    reason=detail.get('reason'), partial=detail.get('partial_observation'),
                    clearance_m=detail.get('swept_clearance_m'), preview=detail.get('turn_preview')))
    return dict(headway_s=headway, calls=calls, changed=changes, zero_to_positive=zero_to_positive,
        positive_to_zero=positive_to_zero, maximum_target_mps=maximum,
        target_time_integral_before_m=sum_old, target_time_integral_after_m=sum_target,
        reason_changes=[dict(before=a,after=b,count=n) for (a,b),n in counts.items()],
        output_counts=dict(output_counts), trace_sha256=sha256(TRACE.read_bytes()).hexdigest(), rows=rows,
        scope='저장 센서·차체 궤적·모터 속도를 고정한 판단 비교. 명령 적분은 실제 진행거리 예측이 아니다.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('mode',choices=('baseline','half_second'))
    args = parser.parse_args()
    result = run(.8 if args.mode == 'baseline' else .5)
    with (HERE/(args.mode+'.json')).open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False))
    if args.mode=='baseline' and result['changed']:
        raise SystemExit(1)
