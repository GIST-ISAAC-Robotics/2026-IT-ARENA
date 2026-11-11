"""충분한 통과 관측 뒤에도 기존 1mm 후반부 위험 분류를 적용하는 사본 비교."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
import arena_autonomy.pass_release_trial as ps


def candidate(owner, scene, baseline, speed, previous, dt, raw_scan):
    target, steer, base = baseline
    partial = base.get('partial_observation') or {}
    if (owner.mode != 'RELEASED' or not owner.follow_config.traffic_recovery_trial
            or not 0 < target or speed < 0
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or base.get('boundary_from_memory') is not False
            or base.get('path_source_s') != scene.source_first_s
            or any(s['present'] or (s['seen'] and (not s['passed'] or s['clear'] < owner.config.release_fresh))
                   for s in owner.sides.values())):
        return None
    target = min(target, owner.config.guard_speed_cap_mps)
    context = {}
    region = ps.motion_region(scene, speed, target, steer, previous, dt,
        owner.follow_config, owner.lat, context=context)
    fringe = ps.rear_fringe_risk(owner, scene, raw_scan, region, context)
    if not fringe['accepted'] or ps.region_blocked(region, owner.config, bounded_unknown_risk=True):
        return None
    return target, steer, dict(region=region, fringe=fringe,
        measured_speed_mps=speed, original_target_mps=baseline[0])


def main():
    out = A/'confirmed_fringe_probe'
    out.mkdir(exist_ok=False)
    path = A/'run1/follow_decisions.jsonl'
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in
        [path, *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original = ps.PassReleaseSelector.select
    counts, rows = Counter(), []
    def select(self, scene, baseline, parallel, speed, previous, dt, **kw):
        result = original(self, scene, baseline, parallel, speed, previous, dt, **kw)
        if result[2].get('selector_reason') == 'large_motion_region_unobserved':
            value = candidate(self, scene, baseline, speed, previous, dt, kw.get('raw_scan'))
            if value is not None:
                counts['candidate_accepts'] += 1
                rows.append(freeze(dict(t=scene.t_ref_s, before=result[:2], after=value[:2],
                    candidate=value[2], original=result[2].get('selector_region'))))
        return result
    ps.PassReleaseSelector.select = select
    try:
        for event in read_events(path):
            data = event['data']
            if event['kind'] == 'header': controller, motion = build_controller(data)
            elif event['kind'] == 'reset': controller.reset()
            elif event['kind'] == 'call':
                result = freeze(replay_call(controller, data['input'], motion))
                counts['calls'] += 1
                counts['command_mismatch'] += not equal_output(result[:2], data['output'][:2])
    finally:
        ps.PassReleaseSelector.select = original
    result = dict(scope='shadow-only proposal with unchanged one-mm/three-degree/near-return gates',
        counts=dict(counts), accepted=rows, sources=sources,
        source_unchanged=all(sha(ROOT/p) == h for p, h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(counts=counts, source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch'] == 0 and result['source_unchanged']


if __name__ == '__main__': main()
