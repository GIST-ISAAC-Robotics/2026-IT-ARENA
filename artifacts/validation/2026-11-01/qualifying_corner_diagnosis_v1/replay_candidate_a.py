"""후보 A 사본의 저장 입력 전 구간 재생. 옵션 OFF 무변화와 ON 작동 범위를 함께 본다.

실행: python3 -I replay_candidate_a.py <run> <snap_candA> <off|on> <out.json>
결합 실행의 정지 뒤(46.40 s~)는 차체가 움직이지 않는 동안의 조향만 되먹임한다.
"""
import bisect
import gzip
import json
import sys
from dataclasses import replace
from pathlib import Path

run, snapshot, mode, out = sys.argv[1:5]
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FOLDER = {'combined_failed': ROOT / 'qualifying_profile_v1/run1', 'rise_only': ROOT / 'race_acceleration_v1/run1',
          'turn_only': ROOT / 'turn_preview_v1/run4'}[run]
for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
    sys.path.insert(0, str(Path(snapshot) / 'src' / pkg))
from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig

STOP_T = 46.40
with gzip.open(HERE / f'extract_{run}.json.gz', 'rt', encoding='utf-8') as f:
    rows = [r for r in json.load(f)['trajectory'] if r['sim_time_s'] is not None]
times, body = [r['sim_time_s'] for r in rows], [r['truth_longitudinal_mps'] for r in rows]


def truth(t):
    return body[min(bisect.bisect_left(times, t), len(body)-1)]


follow = motion = None
compared = 0
changes, stopped, fired, publish_prev, first_positive = [], [], [], 0., None
low_reading = moving_low = 0
with (FOLDER / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        kind, d = r['kind'], r['data']
        if kind == 'header':
            motion = MotionConfig(**d['motion_config'])
            follow = lf.LocalFollow(replace(lf.FollowConfig(**d['follow_config']), low_speed_arc_reacquire=mode == 'on'))
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            i, rec = d['input'], d['output']
            t = i['reference_s']
            if t >= 7.9 and abs(i['speed_mps']) <= .10:
                low_reading += 1
                moving_low += truth(t) >= .25
            feedback = mode == 'on' and run == 'combined_failed' and t >= STOP_T
            if feedback:
                i['previous_steering_rad'] = publish_prev
            v, s, det = replay_call(follow, i, motion)
            if det.get('bilateral_reference') == 'low_speed_arc' or det.get('reason') == 'follow_steer_in_place':
                fired.append(dict(t=round(t, 2), meas=i['speed_mps'], body=round(truth(t), 3), reason=det.get('reason'),
                                  arc=det.get('bilateral_reference') == 'low_speed_arc'))
            if feedback:
                publish_prev = max(publish_prev - .06, min(publish_prev + .06, s))
                stopped.append(dict(t=round(t, 2), v=v, s=round(s, 4), pub=round(publish_prev, 4), reason=det.get('reason')))
                if v > 0:
                    first_positive = stopped[-1]
                    break
            else:
                compared += 1
                if abs(v - rec[0]) > 1e-9 or abs(s - rec[1]) > 1e-9:
                    changes.append(dict(t=round(t, 2), rec=[rec[0], rec[1]], new=[v, s], reason=det.get('reason'), meas=i['speed_mps']))
result = dict(run=run, mode=mode, compared_with_recorded=compared, changed=len(changes), changes=changes[:20],
              first_positive=first_positive, stopped_calls=len(stopped), stopped_reasons=sorted({str(x['reason']) for x in stopped}),
              low_reading_calls=low_reading, low_reading_while_body_ge_0p25=moving_low, branch_events=len(fired),
              branch_events_body_ge_0p25=sum(1 for x in fired if x['body'] >= .25), fired=fired[:20],
              scope='개방 루프 재생. 결합 실행 정지 구간만 발행 조향을 모사했고 차체 이동은 모사하지 않는다.')
with open(out, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(result, f, ensure_ascii=False, indent=1)
print({k: v for k, v in result.items() if k not in ('changes', 'fired', 'scope')})
