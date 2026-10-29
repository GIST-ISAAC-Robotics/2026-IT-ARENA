"""완료된 run1의 순서 있는 입력을 새 구현으로 비교한다. 폐루프 주행이 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1'))
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from safety_shadow import ShadowSafety


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = BASE/'recording_comparison'
    out.mkdir(exist_ok=False)
    trace = BASE.parent/'traffic_recovery_connection_v1/run1_course/follow_decisions.jsonl'
    exact = json.loads((trace.parent/'follow_trace_replay.json').read_text())
    assert exact['passed'] and exact['input_unchanged']
    before = digest(trace)
    assert before == exact['input_sha256']
    sources = sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sources}
    counts, reasons = Counter(), Counter()
    changed, target, costs = [], [], []
    gate = None
    with trace.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        follow = LocalFollow(FollowConfig(**header['follow_config']))
        assert follow.config.traffic_recovery_trial
        motion = MotionConfig(**header['motion_config'])
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                follow.reset()
                counts['reset'] += 1
                continue
            if event['kind'] != 'call':
                continue
            data = event['data']
            stamp = data['input']['reference_s']
            if stamp > 60.:
                break
            start = time.perf_counter()
            candidate = replay_call(follow, data['input'], motion)
            costs.append((time.perf_counter()-start)*1000.)
            counts['calls'] += 1
            if gate is None:
                gate = ShadowSafety(RecoveryPolicy(True, True, True), follow.config,
                                    data['input']['rear_half_angle_deg'])
                assert gate.node.partial_policy is not follow.partial_policy
            safe, status = gate.step(data['input'], motion, candidate[:2])
            assert 0. <= safe[0] <= candidate[0]+1e-12
            if candidate[0] == 0.:
                assert safe[0] == 0.
                counts['zero_request_preserved'] += 1
            changed_command = not np.allclose(candidate[:2], data['output'][:2], atol=1e-10, rtol=0.)
            if changed_command:
                counts['changed_command'] += 1
                reasons[candidate[2].get('selector_reason', candidate[2].get('reason'))] += 1
                changed.append(dict(t_s=stamp, before=data['output'][:2], after=candidate[:2], safe=safe,
                                    reason=candidate[2].get('selector_reason')))
            if 30. <= stamp <= 49.:
                counts['target_calls'] += 1
                counts['target_recorded_positive'] += int(data['output'][0] > 0.)
                counts['target_candidate_positive'] += int(candidate[0] > 0.)
                counts['target_safe_positive'] += int(safe[0] > 0.)
                if abs(stamp-40.) < 1e-7:
                    target.append(dict(t_s=stamp, recorded=data['output'], candidate=candidate,
                                       safe=safe, safety_status=status))
    unchanged = before == digest(trace) and all(digest(ROOT/p) == h for p, h in hashes.items())
    report = dict(scope='saved inputs through 60 s, stateful planner and independent safety function; no physics/ROS loop',
                  input_sha256=before, input_unchanged=unchanged, source_sha256=hashes,
                  counts=dict(counts), changed_reasons=dict(reasons),
                  first_changed=changed[0] if changed else None,
                  planner_cost_ms=dict(median=float(np.median(costs)), p99=float(np.percentile(costs,99)),
                                       maximum=max(costs)),
                  passed=bool(unchanged and target and target[0]['safe'][0] > 0.
                              and counts['target_safe_positive'] > counts['target_recorded_positive']))
    for name, value in [('report', report), ('changed_commands', changed), ('target_frame', target)]:
        (out/(name+'.json')).write_text(json.dumps(freeze(value), ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('passed','counts','changed_reasons','first_changed','planner_cost_ms')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
