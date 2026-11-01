"""현재 소스로 저장 판단을 재생해 옵션 OFF 무변화와 단독 ON 작동 범위를 확인한다.

저장 입력 재생일 뿐 차체 궤적이나 랩 시간을 말하지 않는다.
"""
from dataclasses import replace
import json
from pathlib import Path

from arena_autonomy import follow_trace as ft
from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOLO = {'rise_only': ROOT/'race_acceleration_v1/run1', 'turn_only': ROOT/'turn_preview_v1/run4',
        'combined_failed': ROOT/'qualifying_profile_v1/run1'}
HANDOFF = {'released_rear_sweep': ROOT/'released_rear_sweep_v1/run1'}


def events(folder):
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def solo(folder, flag, stop_s=None):
    follow = motion = None
    calls = changed = reacquire = steer = 0
    first = []
    for event in events(folder):
        kind, data = event['kind'], event['data']
        if kind == 'header':
            motion = MotionConfig(**data['motion_config'])
            follow = lf.LocalFollow(replace(lf.FollowConfig(**data['follow_config']), low_speed_arc_reacquire=flag))
        elif kind == 'reset':
            follow.reset()
        elif kind == 'call':
            now = data['input']['reference_s']
            if stop_s is not None and now >= stop_s:
                break
            v, s, detail = replay_call(follow, data['input'], motion)
            calls += 1
            reacquire += detail.get('bilateral_reference') == 'low_speed_arc'
            steer += detail.get('reason') == 'follow_steer_in_place'
            if abs(v-data['output'][0]) > 1e-9 or abs(s-data['output'][1]) > 1e-9:
                changed += 1
                if len(first) < 5:
                    first.append(dict(t=now, recorded=data['output'][:2], replay=[v, s], reason=detail.get('reason')))
    return dict(option=flag, calls=calls, changed=changed, reacquire_calls=reacquire, steer_in_place_calls=steer, first_changes=first)


def handoff(folder):
    # 이후 소스에서 늘어난 진단 키가 아니라 속도·조향 일치만 비교한다.
    ft.equal_output = lambda a, b, tolerance=1e-9: abs(a[0]-b[0]) <= tolerance and abs(a[1]-b[1]) <= tolerance
    ft.comparable_decision = lambda value: value
    result = ft.replay_events(events(folder))
    return dict(option=False, calls=result['calls'], changed=len(result['mismatches']), scope=result['scope'])


result = dict(scope='저장 입력 개방 루프 재생. 속도·조향 비교.', solo={}, handoff={})
for name, folder in SOLO.items():
    result['solo'][name] = dict(off=solo(folder, False))
    # 결합 실행은 정지 뒤 후보가 달라지는 것이 목적이므로 정지 전까지만 ON을 비교한다.
    result['solo'][name]['on'] = solo(folder, True, 46.30 if name == 'combined_failed' else None)
    print(name, result['solo'][name])
for name, folder in HANDOFF.items():
    result['handoff'][name] = handoff(folder)
    print(name, result['handoff'][name])
(HERE/'replay_saved.json').write_bytes((json.dumps(result, ensure_ascii=False, indent=1)+'\n').encode('utf-8'))
