"""paced 실행의 저장 입력을 closing 설정에 넣어 교착이 후보 논리에서 오는지 가른다.

같은 측정·직전 조향을 쓴 발행 전 비교다. closing으로 주행했다면 같은 궤적이 나왔다는 뜻은 아니며,
같은 관측이 주어졌을 때 두 설정의 판단이 언제부터 같은지만 보여 준다.
"""
from dataclasses import replace
from hashlib import sha256
import json
from pathlib import Path
import sys

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def run(folder):
    report = json.loads((folder / 'report.json').read_bytes())
    green = report['case_window']['green_sim_s']
    digest = sha256()
    rows = []
    with (folder / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            digest.update(line)
            e = json.loads(line)
            d = e['data']
            if e['kind'] == 'header':
                motion = MotionConfig(**d['motion_config'])
                config = FollowConfig(**d['follow_config'])
                params = HandoffExecutionConfig.from_mapping(d['execution_config'])
                recorded = BypassHandoffFollow(config, params, allow_simulation_trial=True)
                closing = BypassHandoffFollow(config, replace(params, low_motion_reserve=False, moving_pace_floor=False),
                                              allow_simulation_trial=True)
            elif e['kind'] == 'reset':
                recorded.reset()
                closing.reset()
            elif e['kind'] == 'call':
                inp = d['input']
                t = inp['reference_s'] - green
                v, a, q = d['output']
                v0, a0, q0 = replay_call(recorded, inp, motion)
                if max(abs(v0 - v), abs(a0 - a)) > 1e-8 or q0.get('handoff_state') != q.get('handoff_state'):
                    raise ValueError(f'기록 출력 재현 실패 seq={e["seq"]}')
                v1, a1, q1 = replay_call(closing, inp, motion)
                rows.append((t, v, v1, abs(a - a1), q.get('handoff_state'), q1.get('handoff_state')))
    if digest.hexdigest() != report['follow_decisions']['sha256']:
        raise ValueError('판단 기록 불일치')
    return rows


def summarize(rows):
    entry = next((r[0] for r in rows if r[4] == 'BYPASS'), None)
    entry_closing = next((r[0] for r in rows if r[5] == 'BYPASS'), None)
    before = [r for r in rows if entry is not None and r[0] < entry]
    after = [r for r in rows if entry is not None and r[0] >= entry]
    return dict(calls=len(rows), paced_bypass_entry_s=entry, closing_bypass_entry_s=entry_closing,
                before_entry=dict(calls=len(before), speed_differences=int(sum(abs(r[1] - r[2]) > 1e-7 for r in before)),
                                  steering_differences=int(sum(r[3] > 1e-8 for r in before)),
                                  state_differences=int(sum(r[4] != r[5] for r in before))),
                after_entry=dict(calls=len(after), speed_differences=int(sum(abs(r[1] - r[2]) > 1e-7 for r in after)),
                                 steering_differences=int(sum(r[3] > 1e-8 for r in after)),
                                 state_differences=int(sum(r[4] != r[5] for r in after)),
                                 closing_zero_calls=int(sum(r[2] <= .001 for r in after))),
                scope='같은 관측을 넣은 발행 전 판단 비교. 실제 closing 주행 궤적이 아니다.')


if __name__ == '__main__':
    result = summarize(run(Path(sys.argv[1])))
    text = json.dumps(result, ensure_ascii=False, indent=2)
    with Path(sys.argv[2]).open('x', encoding='utf-8') as f:
        f.write(text)
    print(json.dumps(result, ensure_ascii=False))
