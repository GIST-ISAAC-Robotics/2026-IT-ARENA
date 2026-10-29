"""저장 입력 재생에서 같은 경로 승인이 거부되는 정확한 값만 계측한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from arena_autonomy import pass_release_trial as selection
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig

BASE = Path(__file__).resolve().parent
trace = BASE / 'run1_course/follow_decisions.jsonl'
def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

original = selection.same_path_risk
counts, reasons, examples = Counter(), Counter(), {}
def instrument(owner, scene, raw_scan, region, context, speed, target, steer, previous, baseline):
    result = original(owner, scene, raw_scan, region, context, speed, target, steer, previous, baseline)
    if result.get('reason') == 'different_or_larger_motion':
        checked = (baseline.get('partial_observation') or {}).get('timed_missing_diagnostic') or {}
        causes = []
        if not 0 < target <= .35: causes.append('target_above_trial_cap')
        if speed < 0: causes.append('negative_speed')
        if max(target, speed) > checked.get('checked_speed_mps', -1) + 1e-9: causes.append('speed_above_proof')
        if abs(steer - checked.get('checked_steering_rad', 99)) > 1e-9: causes.append('different_steering')
        if abs(previous - checked.get('checked_previous_steering_rad', 99)) > 1e-9: causes.append('different_previous')
        key = ','.join(causes)
        counts[key] += 1
        if key not in examples:
            examples[key] = dict(t_s=scene.t_ref_s, speed=speed, target=target, steer=steer,
                                 previous=previous, checked=checked, region=region)
    reasons[result.get('reason')] += 1
    return result

selection.same_path_risk = instrument
before = digest(trace)
calls = mismatches = 0
with trace.open(encoding='utf-8') as stream:
    header = json.loads(next(stream))['data']
    follow = LocalFollow(FollowConfig(**header['follow_config']))
    motion = MotionConfig(**header['motion_config'])
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'reset': follow.reset(); continue
        if row['kind'] != 'call': continue
        calls += 1
        result = freeze(replay_call(follow, row['data']['input'], motion))
        mismatches += not equal_output(comparable_decision(result), comparable_decision(row['data']['output']))
report = dict(scope='read-only instrumentation of identical decision replay', calls=calls,
              mismatches=mismatches, causes=dict(counts), reasons=dict(reasons), examples=examples,
              input_sha256=before, input_unchanged=digest(trace)==before)
with (BASE/'run1_motion_contract.json').open('x', encoding='utf-8') as stream:
    json.dump(freeze(report),stream,ensure_ascii=False,indent=2)
print(json.dumps(report, ensure_ascii=False))
