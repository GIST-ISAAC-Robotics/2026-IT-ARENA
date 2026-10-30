"""기존 자세/센서 입력을 고정한 대체 판단 비교. 새 궤적 주행 결과가 아니다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from candidate import StraightLanePreferenceSelector


def controller(config):
    follow = LocalFollow(config)
    follow.pass_selector = StraightLanePreferenceSelector(config)
    follow.reset()
    return follow


def main(path, output):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    stats, reasons = Counter(), Counter()
    first_changed = last_applied = None
    footer = None
    with path.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))
        if header['seq'] != 0 or header['kind'] != 'header' or header['data']['schema'] != 1:
            raise ValueError('only complete G1 decision input supported')
        motion = MotionConfig(**header['data']['motion_config'])
        config = FollowConfig(**header['data']['follow_config'])
        a, b = controller(config), controller(config)
        with output.with_suffix('.jsonl').open('x', encoding='utf-8') as detail_file:
            for index, line in enumerate(stream, 1):
                event = json.loads(line)
                if event['seq'] != index or footer is not None:
                    raise ValueError('trace sequence invalid')
                if event['kind'] == 'reset':
                    a.reset(); b.reset()
                elif event['kind'] == 'footer':
                    footer = event['data']
                elif event['kind'] == 'call':
                    raw = event['data']
                    actual = freeze(replay_call(a, raw['input'], motion))
                    repeated = freeze(replay_call(b, raw['input'], motion))
                    expected = raw['output']
                    stats['calls'] += 1
                    stats['deterministic'] += equal_output(comparable_decision(actual), comparable_decision(repeated))
                    changed = not equal_output(actual[:2], expected[:2])
                    stats['commands_changed'] += changed
                    stats['new_zero_output'] += expected[0] > 0 and actual[0] == 0
                    stats['zero_to_positive'] += expected[0] == 0 and actual[0] > 0
                    stats['slower_positive'] += 0 < actual[0] < expected[0]-1e-8
                    stats['faster'] += actual[0] > expected[0]+1e-8
                    info = actual[2].get('rear_lane_keep') or {}
                    reasons[str(info.get('reason'))] += 1
                    ref = raw['input']['reference_s']
                    if changed and first_changed is None:
                        first_changed = ref
                    if info.get('applied'):
                        stats['applied'] += 1
                        last_applied = ref
                    detail_file.write(json.dumps(dict(seq=index, reference_s=ref,
                        previous_command=expected[:2], candidate_command=actual[:2],
                        mode=actual[2].get('selector_mode'), info=info), ensure_ascii=False)+'\n')
                else:
                    raise ValueError('unexpected trace event')
    if footer is None or any(footer.get(k) != 0 for k in ('capture_errors','failed','dropped','pending')):
        raise ValueError('incomplete trace')
    report = dict(scope='fixed recorded sensor path, changed command proposals; not closed-loop or ROS replay',
                  source=path.relative_to(ROOT).as_posix(), input_sha256=digest,
                  input_unchanged=digest == hashlib.sha256(path.read_bytes()).hexdigest(),
                  candidate_sha256=hashlib.sha256((BASE/'candidate.py').read_bytes()).hexdigest(),
                  stats=dict(stats), reasons=dict(reasons), first_changed_s=first_changed,
                  last_applied_s=last_applied)
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['input_unchanged'] and stats['deterministic'] == stats['calls'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(main(args.input.resolve(), args.output.resolve()))
