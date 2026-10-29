"""완료된 기록의 기하 역할만 비교한다. 운영 코드와 원래 결과는 변경하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(BASE.parent/'traffic_residual_v1'))
from arena_autonomy import corridor_observer
from arena_autonomy.corridor_roles import role_aware_evidence
from arena_autonomy.follow_trace import equal_output, freeze, replay_call
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from safety_shadow import ShadowSafety


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    trace = BASE.parent/'traffic_release_fringe_v1/run1_course/follow_decisions.jsonl'
    exact = json.loads((trace.parent/'follow_trace_replay.json').read_text())
    before = digest(trace)
    assert exact['passed'] and exact['input_unchanged'] and exact['input_sha256'] == before
    output = BASE/'comparison'
    output.mkdir(exist_ok=False)
    sources = sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sources}
    original = corridor_observer.corridor_evidence
    counts, reasons, changed_reasons, role_counts = Counter(), Counter(), Counter(), Counter()
    changes, target, costs = [], [], []
    gate = None

    def observe_roles(points, preferred, config):
        result = role_aware_evidence(points, preferred, config)
        role_counts[result.get('role_selection', 'missing')] += 1
        role_counts['supported_'+str(bool(result['supported']))] += 1
        return result

    corridor_observer.corridor_evidence = observe_roles
    try:
        with trace.open(encoding='utf-8') as stream:
            header = json.loads(next(stream))['data']
            config = FollowConfig(**header['follow_config'])
            assert config.traffic_recovery_trial and config.corridor_fallback_enabled
            follow, motion = LocalFollow(config), MotionConfig(**header['motion_config'])
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
                start = time.perf_counter()
                candidate = replay_call(follow, data['input'], motion)
                costs.append(1000.*(time.perf_counter()-start))
                if gate is None:
                    gate = ShadowSafety(RecoveryPolicy(True, True, True), config,
                                        data['input']['rear_half_angle_deg'])
                    assert gate.node.partial_policy is not follow.partial_policy
                safe, status = gate.step(data['input'], motion, candidate[:2])
                assert 0. <= safe[0] <= candidate[0]+1e-12
                if candidate[0] == 0.:
                    assert safe[0] == 0.
                    counts['zero_request_preserved'] += 1
                counts['calls'] += 1
                reasons[candidate[2]['reason']] += 1
                different = not equal_output(freeze(candidate[:2]), data['output'][:2])
                if different:
                    counts['changed_commands'] += 1
                    changed_reasons[candidate[2]['reason']] += 1
                    changes.append(dict(t_s=stamp, before=data['output'][:2], after=candidate[:2],
                                        safe=safe, reason=candidate[2]['reason'],
                                        selector=candidate[2].get('selector_reason'),
                                        corridor=candidate[2].get('corridor_candidate')))
                if 129.76 <= stamp <= 154.:
                    counts['late_calls'] += 1
                    counts['late_recorded_positive'] += int(data['output'][0]>0.)
                    counts['late_candidate_positive'] += int(candidate[0]>0.)
                    counts['late_safe_positive'] += int(safe[0]>0.)
                if any(abs(stamp-t)<1e-7 for t in (129.5,129.76,130.,131.,136.,150.)):
                    target.append(dict(t_s=stamp, recorded=data['output'], candidate=candidate,
                                       safe=safe, safety_status=status))
    finally:
        corridor_observer.corridor_evidence = original
    source_ok = all(digest(ROOT/p)==value for p,value in hashes.items())
    input_ok = digest(trace)==before
    report = dict(scope='stateful recorded-input role comparison and independent safety function; no ROS or physics loop',
        input_sha256=before, input_unchanged=input_ok, source_unchanged=source_ok, source_sha256=hashes,
        baseline_exact_calls=exact.get('matched'), counts=dict(counts), reasons=dict(reasons),
        changed_reasons=dict(changed_reasons), role_counts=dict(role_counts),
        first_changed=changes[0] if changes else None,
        function_restored=corridor_observer.corridor_evidence is original,
        runtime_changed=False, branch_semantics_certified=False,
        cost_ms=dict(median=float(np.median(costs)),p99=float(np.percentile(costs,99)),maximum=max(costs)),
        comparison_valid=source_ok and input_ok,
        late_improvement=counts['late_safe_positive']>counts['late_recorded_positive'])
    for name,value in [('report',report),('changed_commands',changes),('target_frames',target)]:
        (output/(name+'.json')).write_text(json.dumps(freeze(value),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},ensure_ascii=False))
    return 0 if report['comparison_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
