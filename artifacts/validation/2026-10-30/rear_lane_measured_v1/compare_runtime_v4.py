"""연결된 observed_measured_v1이 10/29 오프라인 v4와 같은 고정 입력에서 같은 명령을 내는지 대조한다.

입력은 v3 observed Gazebo 실행의 652호출 기록이며, 비교 대상은 v4 shadow_gazebo.jsonl의 후보 명령이다.
새 궤적·폐루프 결과가 아니다. 목표 상한 경계 수정은 최대속도 1.0 설정에서 출력에 영향이 없어야 한다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

INPUT = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_gazebo_v1/drive_on/follow_decisions.jsonl'
SHADOW = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4/shadow_gazebo.jsonl'


def main(output):
    digest = hashlib.sha256(INPUT.read_bytes()).hexdigest()
    shadow = {row['seq']: row for row in map(json.loads, SHADOW.read_text(encoding='utf-8').splitlines())}
    stats, reasons, mismatches = Counter(), Counter(), []
    with INPUT.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))
        motion = MotionConfig(**header['data']['motion_config'])
        recorded = header['data']['follow_config']
        assert recorded['rear_lane_keep_policy'] == 'observed'
        config = FollowConfig(**{**recorded, 'rear_lane_keep_policy': 'observed_measured_v1'})
        a, b = LocalFollow(config), LocalFollow(config)
        for index, line in enumerate(stream, 1):
            event = json.loads(line)
            assert event['seq'] == index
            if event['kind'] == 'reset':
                a.reset(); b.reset()
            elif event['kind'] == 'call':
                raw = event['data']
                actual = freeze(replay_call(a, raw['input'], motion))
                repeated = freeze(replay_call(b, raw['input'], motion))
                stats['calls'] += 1
                stats['deterministic'] += equal_output(comparable_decision(actual), comparable_decision(repeated))
                expected = shadow[index]['candidate_command']
                same = equal_output(list(actual[:2]), expected)
                stats['matches_offline_v4'] += same
                stats['differs_from_recorded_v3'] += not equal_output(actual[:2], raw['output'][:2])
                info = actual[2].get('rear_lane_keep') or {}
                reasons[str(info.get('reason'))] += 1
                stats['applied'] += bool(info.get('applied'))
                if not same and len(mismatches) < 5:
                    mismatches.append(dict(seq=index, runtime=list(actual[:2]), offline=expected))
    report = dict(scope='fixed recorded inputs; runtime observed_measured_v1 vs offline v4 commands; not closed loop',
                  input=INPUT.relative_to(ROOT).as_posix(), input_sha256=digest,
                  input_unchanged=digest == hashlib.sha256(INPUT.read_bytes()).hexdigest(),
                  offline=SHADOW.relative_to(ROOT).as_posix(),
                  offline_sha256=hashlib.sha256(SHADOW.read_bytes()).hexdigest(),
                  stats=dict(stats), reasons=dict(reasons), mismatches=mismatches)
    report['passed'] = (report['input_unchanged'] and stats['calls'] == len(shadow) == 652
                        and stats['deterministic'] == stats['calls'] == stats['matches_offline_v4'])
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    raise SystemExit(main(parser.parse_args().output.resolve()))
