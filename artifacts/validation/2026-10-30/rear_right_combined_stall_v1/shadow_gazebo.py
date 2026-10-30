"""저장된 Gazebo 판단 입력을 고정하고 후보 평행 후보로 대안 명령을 계산한다(개루프, 새 궤적 아님).

먼저 런타임 정책으로 같은 입력을 다시 계산해 기록 명령과의 동일성을 확인한 뒤, 후보와의 차이(0→양수,
양수→0, 감속/가속)를 분리한다. 후보 명령이 실제로 실행됐다면 이후 입력이 달라지므로 첫 변화 이후는 참고값이다.
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
from candidate_state import MeasuredParallelTrial as CandidateTrial


def controllers(config):
    runtime = LocalFollow(config)
    a, b = LocalFollow(config), LocalFollow(config)
    for follow in (a, b):
        follow.parallel_trial = CandidateTrial(follow.config)
        follow.reset()
    return runtime, a, b


def main(path, output):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    stats, reasons = Counter(), Counter()
    first_change, first_zero_to_positive, changes = None, None, []
    with path.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        config = FollowConfig(**header['follow_config'])
        assert config.rear_lane_keep_policy == 'observed_measured_v1'
        runtime, a, b = controllers(config)
        for index, line in enumerate(stream, 1):
            event = json.loads(line)
            if event['kind'] == 'reset':
                for c in (runtime, a, b):
                    c.reset()
                continue
            if event['kind'] != 'call':
                continue
            raw = event['data']
            base = freeze(replay_call(runtime, raw['input'], motion))
            cand = freeze(replay_call(a, raw['input'], motion))
            again = freeze(replay_call(b, raw['input'], motion))
            expected = raw['output']
            t = raw['input']['reference_s']
            stats['calls'] += 1
            stats['runtime_matches_record'] += equal_output(base[:2], expected[:2])
            stats['candidate_deterministic'] += equal_output(comparable_decision(cand), comparable_decision(again))
            changed = not equal_output(cand[:2], expected[:2])
            stats['changed'] += changed
            stats['zero_to_positive'] += expected[0] == 0 and cand[0] > 0
            stats['positive_to_zero'] += expected[0] > 0 and cand[0] == 0
            stats['slower_positive'] += 0 < cand[0] < expected[0]-1e-8
            stats['faster_positive'] += cand[0] > expected[0]+1e-8 and expected[0] > 0
            trial = cand[2].get('parallel_hold_trial') or {}
            stats['beside_age_only_calls'] += bool(trial.get('beside_age_only_points'))
            if changed:
                reasons[f"{expected[2].get('reason')} -> {cand[2].get('reason')}"] += 1
                first_change = t if first_change is None else first_change
                if len(changes) < 12:
                    changes.append(dict(t=t, recorded=list(expected[:2]), candidate=list(cand[:2]),
                                        recorded_reason=expected[2].get('reason'), candidate_reason=cand[2].get('reason')))
            if expected[0] == 0 and cand[0] > 0 and first_zero_to_positive is None:
                first_zero_to_positive = t
    report = dict(scope='fixed recorded Gazebo inputs; open-loop alternative commands, not a new trajectory',
                  input=path.relative_to(ROOT).as_posix(), input_sha256=digest,
                  input_unchanged=digest == hashlib.sha256(path.read_bytes()).hexdigest(),
                  stats=dict(stats), change_reasons=dict(reasons.most_common(12)),
                  first_change_s=first_change, first_zero_to_positive_s=first_zero_to_positive, first_changes=changes)
    report['passed'] = (report['input_unchanged'] and stats['calls'] == stats['runtime_matches_record']
                        == stats['candidate_deterministic'])
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
