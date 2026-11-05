"""저장 입력 재생(평가 전용). PYTHONPATH로 고른 격리 트리(base/cand)의 제어기로 판단 기록을 다시 계산한다.

사용: replay_check.py TAG TRACE [--straight-budget]  → replay_TAG.jsonl.gz, replay_TAG.json
각 호출의 (속도, 조향, 사유), 전체 판단값 SHA-256(표면 정합 처리시간 제외), 기록과의 일치, 호출 처리시간을 남긴다.
--straight-budget은 기록 헤더의 FollowConfig에 solo_straight_budget_trial=True만 더한다(단독 구성 전용).
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import time

import arena_autonomy
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow

HERE = Path(__file__).resolve().parent


def build(header, straight):
    follow_config = dict(header['follow_config'])
    if straight:
        follow_config['solo_straight_budget_trial'] = True
    config = FollowConfig(**follow_config)
    schema = header.get('schema')
    if schema == 1:
        return LocalFollow(config)
    if schema == 5:
        from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
        return BypassHandoffFollow(config, HandoffExecutionConfig.from_mapping(header['execution_config']),
                                   allow_simulation_trial=True)
    raise SystemExit(f'unsupported trace schema {schema}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('tag')
    parser.add_argument('trace', type=Path)
    parser.add_argument('--straight-budget', action='store_true')
    args = parser.parse_args()
    follow = motion = header = None
    calls = matched = 0
    footer = None
    rows = []
    with args.trace.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            kind, data = event['kind'], event['data']
            if kind == 'header':
                header = data
                motion = MotionConfig(**data['motion_config'])
                follow = build(data, args.straight_budget)
            elif kind == 'reset':
                follow.reset()
            elif kind == 'call':
                start = time.perf_counter()
                actual = freeze(replay_call(follow, data['input'], motion))
                cost = time.perf_counter()-start
                comparable = comparable_decision(actual)
                same = equal_output(comparable, comparable_decision(data['output']))
                calls += 1
                matched += same
                detail = actual[2]
                rows.append(dict(seq=event['seq'], t=data['input']['reference_s'], v=actual[0], steer=actual[1],
                                 reason=detail.get('reason'), behavior=detail.get('behavior'),
                                 budget=detail.get('solo_straight_budget'), original=detail.get('original_speed_mps'),
                                 measured=data['input']['speed_mps'], recorded_v=data['output'][0],
                                 digest=hashlib.sha256(json.dumps(comparable, sort_keys=True).encode()).hexdigest(),
                                 same_as_recorded=same, cost_s=cost))
            elif kind == 'footer':
                footer = data
    with gzip.open(HERE/f'replay_{args.tag}.jsonl.gz', 'wt', encoding='utf-8') as out:
        for row in rows:
            out.write(json.dumps(row)+'\n')
    summary = dict(tag=args.tag, trace=str(args.trace), module=arena_autonomy.__file__, straight_budget=args.straight_budget,
                   schema=header.get('schema'), calls=calls, matched_recorded=matched, footer_present=footer is not None,
                   footer_capture_errors=None if footer is None else footer.get('capture_errors'))
    (HERE/f'replay_{args.tag}.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1)+'\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
