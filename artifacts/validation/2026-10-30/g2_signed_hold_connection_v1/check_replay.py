"""v1 기록 동일성과 오프라인 v2/명시 런타임 v2의 같은 입력 출력을 비교한다."""
from dataclasses import replace
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(BASE.parent/'g2_signed_hold_v1'), str(ROOT/'src/arena_autonomy'),
    str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from shadow import build
from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def main():
    reports = []
    for name in ('g2_center_gazebo_v1', 'g2_center_stress_v1'):
        events = [json.loads(line) for line in (BASE.parent/name/'run1/g2_decisions.jsonl').open()]
        header = events[0]['data']
        before, offline = build(header, False), build(header, True)
        runtime = BypassEntryFollow(FollowConfig(**header['follow_config']),
            replace(BypassExecutionConfig(**header['execution_config']), corridor_policy='corridor_v2_signed_hold'),
            allow_simulation_trial=True)
        motion = MotionConfig(**header['motion_config'])
        total = baseline = matched = 0
        for event in events[1:]:
            if event['kind'] == 'reset':
                for f in (before, offline, runtime): f.reset()
            elif event['kind'] == 'call':
                d = event['data']; total += 1
                baseline += equal_output(freeze(replay_call(before, d['input'], motion)), d['output'])
                matched += equal_output(freeze(replay_call(offline, d['input'], motion)),
                                        freeze(replay_call(runtime, d['input'], motion)))
        reports.append(dict(trace=name, calls=total, baseline_matched=baseline, candidate_matched=matched))
    result = dict(passed=all(r['calls'] == r['baseline_matched'] == r['candidate_matched'] for r in reports), reports=reports)
    (BASE/'replay1.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
