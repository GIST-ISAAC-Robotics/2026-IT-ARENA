"""고정 입력에서 원래 판단 재현과 signed 대기 후보 비교. 새 주행/폐루프 성공이 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(BASE), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from candidate import SignedHoldGuide
from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def build(header, candidate):
    follow = BypassEntryFollow(FollowConfig(**header['follow_config']),
        BypassExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    if candidate:
        follow.guide = SignedHoldGuide(follow.config, follow.guide.registered_entry,
            stationary_outer_boundary=True, wall_reanchor=True, tracked_wall_fit=True, rejoin_enabled=True)
    return follow


def run(trace):
    events = [json.loads(line) for line in trace.open()]
    header = events[0]['data']
    motion = MotionConfig(**header['motion_config'])
    old, new = build(header, False), build(header, True)
    match = total = changes = positive_recovered = negative_positive = 0
    reasons = Counter()
    rows = []
    for e in events[1:]:
        if e['kind'] == 'reset':
            old.reset(); new.reset()
        elif e['kind'] == 'call':
            d = e['data']; inp = d['input']
            before = replay_call(old, inp, motion)
            after = replay_call(new, inp, motion)
            total += 1
            match += equal_output(freeze(before), d['output'])
            changed = not equal_output(freeze(before[:2]), freeze(after[:2]))
            changes += changed
            positive_recovered += before[0] == 0 and after[0] > 0
            negative_positive += inp['speed_mps'] < 0 and after[0] > 0
            reasons[after[2]['reason']] += 1
            if changed or after[2]['reason'].startswith('entry_reverse'):
                rows.append(dict(t=inp['reference_s'], speed=inp['speed_mps'], before=before[:2], after=after[:2],
                    reason=after[2]['reason'], underlying=after[2].get('underlying_reason'),
                    forward_confirmations=after[2].get('forward_confirmations')))
    name = trace.parent.parent.name
    (BASE/(name+'_rows.json')).write_text(json.dumps(rows)+'\n')
    return dict(trace=str(trace.relative_to(ROOT)), trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        calls=total, baseline_matched=match, command_changes=changes, positive_recovered=positive_recovered,
        positive_under_negative_feedback=negative_positive, reasons=dict(reasons), first_events=rows[:4])


def main():
    paths = [*BASE.glob('*.py'), *BASE.glob('*.sh')]
    paths += list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(BASE/'shadow1_sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:
            archive.write(ROOT/name, name)
    reports = [run(BASE.parent/name/'run1/g2_decisions.jsonl') for name in ('g2_center_gazebo_v1', 'g2_center_stress_v1')]
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p,h in hashes.items())
    result = dict(scope='open-loop recorded input; changed commands do not imply an actual driven path',
        reports=reports, source_sha256=hashes, source_unchanged=unchanged)
    (BASE/'shadow1.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(reports=reports, source_unchanged=unchanged), ensure_ascii=False))
    return 0 if unchanged and all(r['calls'] == r['baseline_matched'] and r['positive_under_negative_feedback'] == 0 for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
