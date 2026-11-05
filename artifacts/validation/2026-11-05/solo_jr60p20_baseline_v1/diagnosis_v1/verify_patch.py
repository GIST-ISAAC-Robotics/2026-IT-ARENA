"""candidate_A.patch를 적용한 격리 사본으로 run1 저장 입력을 재생한다(평가 전용).

OFF(기록 헤더 그대로): 1,779호출 모두 기록과 같아야 한다.
ON(bridge_release_continuity=True): 기록과 달라진 호출이 replay.json의 메모리 후보 A 변경과 같아야 한다.
"""
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_call, replay_events, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'run1'


def events():
    with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def main():
    off = replay_events(events())
    follow = motion = None
    changed = []
    for e in events():
        if e['kind'] == 'header':
            motion = MotionConfig(**e['data']['motion_config'])
            follow = LocalFollow(FollowConfig(**{**e['data']['follow_config'], 'bridge_release_continuity': True}))
        elif e['kind'] == 'reset':
            follow.reset()
        elif e['kind'] == 'call':
            actual = freeze(replay_call(follow, e['data']['input'], motion))
            if not equal_output(comparable_decision(actual), comparable_decision(e['data']['output'])):
                changed.append(dict(t=round(e['data']['input']['reference_s'], 3), recorded=e['data']['output'][0],
                                    candidate=actual[0], candidate_reason=actual[2].get('reason'),
                                    steer_delta=actual[1]-e['data']['output'][1]))
    memory = json.loads((HERE/'replay.json').read_bytes())['candidate_A']['changed']
    same = (len(changed) == len(memory) and all(
        a['t'] == b['t'] and abs(a['candidate']-b['candidate']) < 1e-12 and abs(a['steer_delta']) < 1e-12 for a, b in zip(changed, memory)))
    result = dict(scope=__doc__.strip(), off=dict(calls=off['calls'], matched=off['matched'], passed=off['passed']),
                  on_changed=changed, on_matches_memory_candidate=same)
    (HERE/'verify_patch.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'scope'}, ensure_ascii=False))
    return 0 if off['passed'] and same else 1


if __name__ == '__main__':
    raise SystemExit(main())
