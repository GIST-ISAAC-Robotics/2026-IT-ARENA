"""저장 판단 입력을 고정하고 런타임 재현·후보 결정론·바뀐 명령을 분리한다(개루프 대안 계산, 새 궤적 아님).

1) 런타임 observed_measured_v1로 다시 계산한 명령이 기록과 같은지(원본 재현).
2) 후보 두 인스턴스의 판단이 같은지(결정론).
3) 후보와 기록의 명령 차이(0→양수, 양수→0, 속도/조향 변화)와 기억 사건(보존/거절 이유).
첫 변화 이후 입력은 기록 궤적의 것이므로 이후 차이는 참고값이다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(BASE), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from candidate_state import FreshMemoryParallelTrial


def follows(config):
    runtime = LocalFollow(config)
    a, b = LocalFollow(config), LocalFollow(config)
    for follow in (a, b):
        follow.parallel_trial = FreshMemoryParallelTrial(follow.config)
        follow.reset()
    return runtime, a, b


def main(path, output):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    stats, reasons, memory_reasons = Counter(), Counter(), Counter()
    first_change = first_zero_to_positive = None
    changes = []
    with path.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        recorded = FollowConfig(**header['follow_config'])
        # 10/29 v4 합성 기록은 timed 헤더 위에서 같은 조합을 직접 교체했다. 런타임 명시 정책으로 맞춘다.
        config = replace(recorded, rear_lane_keep_trial=True, rear_lane_keep_policy='observed_measured_v1')
        runtime, a, b = follows(config)
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                for follow in (runtime, a, b):
                    follow.reset()
                continue
            if event['kind'] != 'call':
                continue
            raw = event['data']
            base = freeze(replay_call(runtime, raw['input'], motion))
            cand = freeze(replay_call(a, raw['input'], motion))
            again = freeze(replay_call(b, raw['input'], motion))
            expected, t = raw['output'], raw['input']['reference_s']
            stats['calls'] += 1
            stats['runtime_matches_record'] += equal_output(base[:2], expected[:2])
            stats['candidate_deterministic'] += equal_output(comparable_decision(cand), comparable_decision(again))
            memory = cand[2].get('fresh_confirmation_memory')
            if memory:
                stats['memory_evaluations'] += 1
                stats['memory_kept'] += bool(memory['kept'])
                if not memory['kept']:
                    failed = [k for k, v in (memory.get('checks') or {}).items() if not v] or [memory.get('why')]
                    memory_reasons['+'.join(map(str, failed))] += 1
            changed = not equal_output(cand[:2], expected[:2])
            stats['changed'] += changed
            stats['zero_to_positive'] += expected[0] == 0 and cand[0] > 0
            stats['positive_to_zero'] += expected[0] > 0 and cand[0] == 0
            stats['speed_changed_positive'] += expected[0] > 0 and cand[0] > 0 and abs(cand[0]-expected[0]) > 1e-9
            stats['steering_only_changed'] += changed and cand[0] == expected[0]
            trial = cand[2].get('parallel_hold_trial') or {}
            stats['candidate_positive_while_ineligible'] += cand[0] > 0 and trial.get('eligible') is False \
                and cand[2].get('selector_choice') == 'parallel'
            if changed:
                reasons[f"{expected[2].get('reason')} -> {cand[2].get('reason')}"] += 1
                first_change = t if first_change is None else first_change
                if len(changes) < 12:
                    changes.append(dict(t=t, recorded=list(expected[:2]), candidate=list(cand[:2]),
                                        recorded_reason=expected[2].get('reason'), candidate_reason=cand[2].get('reason')))
            if expected[0] == 0 and cand[0] > 0 and first_zero_to_positive is None:
                first_zero_to_positive = t
    report = dict(scope='fixed recorded inputs; open-loop alternative commands; not a new trajectory or Gazebo result',
                  input=path.relative_to(ROOT).as_posix(), input_sha256=digest,
                  input_unchanged=digest == hashlib.sha256(path.read_bytes()).hexdigest(),
                  recorded_policy=recorded.rear_lane_keep_policy, stats=dict(stats),
                  memory_not_kept_reasons=dict(memory_reasons.most_common()), change_reasons=dict(reasons.most_common(12)),
                  first_change_s=first_change, first_zero_to_positive_s=first_zero_to_positive, first_changes=changes)
    report['passed'] = (report['input_unchanged'] and stats['calls'] == stats['runtime_matches_record']
                        == stats['candidate_deterministic'] and stats['candidate_positive_while_ineligible'] == 0)
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != 'first_changes'}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(main(args.input.resolve(), args.output.resolve()))
