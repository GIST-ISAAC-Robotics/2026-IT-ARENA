"""저장 입력 원인 분리만 수행한다. 미완성 교집합 가정 포함, ROS/주행 연결 금지."""
import argparse
from collections import Counter
import difflib
import hashlib
import json
from pathlib import Path
import sys
import time
import types
from unittest.mock import patch
import zipfile

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import arena_autonomy.pass_release_trial as selection
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
import probe_joint_missing_replay as joint


def shadow_class():
    source = (ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py').read_text(encoding='utf-8')
    old = "side['passed'] = side['vanish'] in ('benign', 'rear', 'unknown') and \\\n                    side['clear'] >= self.config.release_fresh"
    new = "side['passed'] = side['passed'] or (side['vanish'] in ('benign', 'rear', 'unknown') and \\\n                    side['clear'] >= self.config.release_fresh)"
    assert source.count(old) == 1
    changed = source.replace(old, new)
    anchor = "        excursion = lateral_excursion(max(speed, b_speed), age, b_steer, previous, dt, self.follow_config)"
    inserted = """        retained = any(s['passed'] and not s['present'] and s['clear'] < self.config.release_fresh
                       for s in self.sides.values())
        detail['shadow_retained_passage'] = retained
        if retained:
            b_speed = min(b_speed, self.config.guard_speed_cap_mps)
""" + anchor
    assert changed.count(anchor) == 1
    changed = changed.replace(anchor, inserted)
    anchor = '        if excursion <= self.config.small_excursion_m:'
    assert changed.count(anchor) == 1
    changed = changed.replace(anchor, '        if excursion <= self.config.small_excursion_m and not retained:')
    module = types.ModuleType('pass_continuity_diagnostic')
    module.__file__ = str(ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py')
    sys.modules[module.__name__] = module
    exec(compile(changed, module.__file__, 'exec'), module.__dict__)
    return module.PassReleaseSelector, source, changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--until', type=float, default=132.)
    args = parser.parse_args()
    if not 130. <= args.until <= 134.:
        raise ValueError('bounded first-stall diagnostic only')
    output = args.output.resolve()
    if not output.is_relative_to(BASE) or output.exists():
        raise ValueError('fresh artifact output required')
    cls, original, changed = shadow_class()
    output.mkdir()
    trace = ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl'
    files = [Path(__file__), ROOT/'scripts/probe_joint_missing_replay.py', ROOT/'scripts/probe_joint_missing_sweep.py',
             *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    trace_hash = hashlib.sha256(trace.read_bytes()).hexdigest()
    with zipfile.ZipFile(output/'source.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in files:
            archive.write(p, str(p.relative_to(ROOT)))
        archive.writestr('effective_selector.py', changed)
        archive.writestr('selector.diff', ''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True))))
    counts, reasons = Counter(), Counter()
    changes, samples, costs = [], [], []
    with trace.open() as stream, patch.object(selection, 'PassReleaseSelector', cls), \
            patch.object(joint.policy_module.PartialObservationPolicy, 'evaluate', joint.evaluate_joint):
        header = json.loads(next(stream))['data']
        follow = LocalFollow(FollowConfig(**header['follow_config']))
        motion = MotionConfig(**header['motion_config'])
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'reset':
                follow.reset()
                counts['resets'] += 1
                continue
            if row['kind'] != 'call':
                continue
            data = row['data']
            stamp = data['input']['reference_s']
            if stamp > args.until:
                break
            began = time.perf_counter()
            result = replay_call(follow, data['input'], motion)
            costs.append((time.perf_counter()-began)*1000.)
            detail = result[2]
            counts['calls'] += 1
            reasons[detail.get('selector_reason', detail['reason'])] += 1
            different = not np.allclose(result[:2], data['output'][:2], rtol=0., atol=1e-10)
            if different:
                counts['changed'] += 1
                if len(changes) < 40:
                    changes.append(dict(t_s=stamp, original=data['output'][:2], candidate=result[:2],
                                        detail=detail))
            if 129.3 <= stamp <= 132. and (not samples or stamp-samples[-1]['t_s'] >= .099):
                samples.append(dict(t_s=stamp, command=result[:2], detail=detail))
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    unchanged = unchanged and hashlib.sha256(trace.read_bytes()).hexdigest() == trace_hash
    report = dict(scope='causal diagnostic only, inherited joint geometry omits per-ray target timing; not deployable or safety/drive proof',
        counts=dict(counts), reasons=dict(reasons), first_change=changes[0] if changes else None,
        source_sha256=hashes, trace_sha256=trace_hash, source_unchanged=unchanged,
        cost_ms=dict(median=float(np.median(costs)), maximum=float(max(costs))))
    for name, value in [('report', report), ('changes', changes), ('samples', samples)]:
        (output/f'{name}.json').write_text(json.dumps(freeze(value), ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(freeze({k: report[k] for k in ('counts', 'first_change', 'source_unchanged', 'cost_ms')})))
    return 0 if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
