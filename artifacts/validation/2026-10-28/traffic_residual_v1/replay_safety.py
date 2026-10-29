"""결합 후보 명령을 독립 보호 함수에 전달하는 제한 저장 입력 비교."""
from collections import Counter
import cProfile
import hashlib
import json
from pathlib import Path
import sys
import time
import pstats
from unittest.mock import patch
import zipfile
import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'),
               str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy import partial_observation as policy
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
import arena_autonomy.pass_release_trial as selection
from recovery_policy import recovery_policy_class
from selector_shadow import selector_class
from safety_shadow import ShadowSafety


def main():
    out = BASE/'safety_replay_v1'
    out.mkdir(exist_ok=False)
    trace = ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    cls = recovery_policy_class()
    selector, _, selector_source, gap_source = selector_class()
    files = [Path(__file__), BASE/'safety_shadow.py', BASE/'recovery_policy.py',
             BASE/'selector_shadow.py', ROOT/'scripts/probe_surface_anchor.py',
             ROOT/'scripts/probe_same_path_gap.py', ROOT/'scripts/probe_timed_policy.py',
             ROOT/'scripts/probe_timed_missing_sweep.py', ROOT/'scripts/probe_joint_missing_sweep.py',
             ROOT/'artifacts/validation/2026-10-27/obstacle_latch_v1/probe_pass_continuity.py',
             *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    trace_hash = hashlib.sha256(trace.read_bytes()).hexdigest()
    with zipfile.ZipFile(out/'source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in files: archive.write(p, p.relative_to(ROOT).as_posix())
        archive.writestr('effective_selector.py', selector_source)
        archive.writestr('effective_gap.py', gap_source)
    counts = Counter()
    reasons = {name: Counter() for name in ('existing', 'recovery')}
    cost = {name: [] for name in ('planner', 'existing', 'recovery')}
    profile=cProfile.Profile()
    profiling=False
    targets = []
    original = policy.PartialObservationPolicy
    with patch.object(policy, 'PartialObservationPolicy', cls), patch.object(selection, 'PassReleaseSelector', selector), trace.open() as stream:
        header = json.loads(next(stream))['data']
        config = FollowConfig(**header['follow_config'])
        follow = LocalFollow(config)
        motion = MotionConfig(**header['motion_config'])
        gates = None
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'reset':
                follow.reset()
                counts['planner_resets'] += 1
                # 제어기 reset은 실제 독립 보호 노드 reset 명령이 아니다.
                continue
            if row['kind'] != 'call': continue
            d = row['data']; stamp = d['input']['reference_s']
            if stamp > 132.: break
            if stamp>=129.3 and not profiling:
                profile.enable();profiling=True
            if gates is None:
                gates = {name: ShadowSafety(kind(True, True, True), config,
                    d['input']['rear_half_angle_deg'])
                    for name, kind in (('existing', original), ('recovery', cls))}
                assert gates['existing'].node.partial_policy is not follow.partial_policy
                assert gates['recovery'].node.partial_policy is not follow.partial_policy
            started = time.perf_counter()
            command = replay_call(follow, d['input'], motion)
            cost['planner'].append((time.perf_counter()-started)*1000)
            counts['calls'] += 1
            target = dict(t_s=stamp, original=d['output'][:2], candidate=command[:2], safety={})
            for name, gate in gates.items():
                started = time.perf_counter()
                output, report = gate.step(d['input'], motion, command[:2])
                cost[name].append((time.perf_counter()-started)*1000)
                reasons[name][report['reason']] += 1
                if output[0] > command[0]+1e-12 or output[0] < 0:
                    raise AssertionError('safety amplified or reversed command')
                if command[0] == 0:
                    assert output[0] == 0
                    counts[name+'_zero_preserved'] += 1
                if stamp >= 129.3:
                    if command[0] > 0: counts['target_'+name+'_requested'] += 1
                    if output[0] > 0: counts['target_'+name+'_positive'] += 1
                    partial = report.get('partial_observation') or {}
                    diagnostic = partial.get('anchor_diagnostic') or {}
                    if diagnostic.get('released'): counts['target_'+name+'_anchor_release'] += 1
                    target['safety'][name] = dict(output=output, reason=report['reason'],
                        clearance=report['swept_clearance_m'], partial=partial)
            if stamp >= 129.3: targets.append(target)
    if profiling:
        profile.disable()
        profile.dump_stats(str(out/'target_profile.pstats'))
        with (out/'target_profile.txt').open('w',encoding='utf-8') as stream:
            pstats.Stats(profile,stream=stream).sort_stats('cumulative').print_stats(45)
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    unchanged &= hashlib.sha256(trace.read_bytes()).hexdigest()==trace_hash
    report = dict(scope='independent safety control function at planner-recorded times; no ROS execution',
        limitations=['same-time delivery assumed; safety receive history unavailable',
                     'only planner-recorded calls, not every safety timer or pre-start scan',
                     'planner reset does not reset safety', 'open-loop stopped trajectory',
                     'static-background matching remains an assumption, not object identity'],
        counts=dict(counts), reasons={k: dict(v) for k,v in reasons.items()},
        cost_ms={k: dict(median=float(np.median(v)), p99=float(np.percentile(v,99)), maximum=max(v))
                 for k,v in cost.items()}, source_sha256=hashes, trace_sha256=trace_hash,
        cost_scope='129.3..132 s calls are cProfile-instrumented; not a rate acceptance benchmark',
        source_unchanged=unchanged)
    for name, value in [('report',report), ('target_calls',targets)]:
        (out/(name+'.json')).write_text(json.dumps(freeze(value),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('counts','cost_ms','source_unchanged')}))
    return 0 if unchanged else 1


if __name__=='__main__': raise SystemExit(main())
