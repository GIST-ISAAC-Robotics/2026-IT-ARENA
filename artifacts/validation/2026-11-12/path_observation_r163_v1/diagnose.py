"""출력/상태를 바꾸지 않고 전역 결측 거절과 현재 경로 영역을 대조한다."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.pass_release_trial import motion_region, region_blocked
from arena_autonomy.known_structure import parse_sectors

A = Path(__file__).resolve().parent
TRACE = A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'
rows = []
seq = None
old_path = LocalFollow._path_command
old_partial = LocalFollow._partial_command
adjacent = None


def path(owner, *args, **kwargs):
    out = old_path(owner, *args, **kwargs)
    owner._r163_path = out
    return out


def partial(owner, scene, history, speed, side, previous=0., dt=.02, raw_scan=None):
    owner._r163_path = None
    out = old_partial(owner, scene, history, speed, side, previous, dt, raw_scan)
    planned = owner._r163_path
    proof = out[2].get('partial_observation') or {}
    if planned is not None and proof.get('allowed') is False:
        row = dict(seq=seq, time_s=scene.t_ref_s, measured_speed_mps=speed,
                   partial_reason=proof.get('reason'), path_speed_mps=planned[0],
                   path_reason=planned[2].get('reason'), path_current=(
                       planned[2].get('boundary_from_memory') is False and
                       planned[2].get('path_source_s') == scene.source_first_s),
                   path_steering_rad=planned[1], latch_reason=proof.get('latch_reason'))
        if planned[0] > 0 and owner.pass_selector is not None:
            target = min(.15, planned[0])
            selector = owner.pass_selector
            context = {}
            region = motion_region(scene, speed, target, planned[1], previous, dt,
                                   owner.config, selector.lat, context=context)
            row.update(target_mps=target, region=region,
                       region_block=region_blocked(region, selector.config))
            if context:
                q = context['queries'][~context['free'] & ~context['masked']]
                proof = adjacent(scene, raw_scan, q, context['padding'], owner.config.length_m,
                    owner.config.width_m, speed, target,
                    parse_sectors(owner.config.known_structure_sectors_json))
                row['local_unknown_test'] = proof
                row['unobserved_bounds_xy_m'] = [np.min(q, axis=0).tolist(), np.max(q, axis=0).tolist()] if len(q) else []
                if seq == 632:
                    geometry = dict(queries=context['queries'].tolist(), free=context['free'].tolist(),
                        masked=context['masked'].tolist(), endpoints=np.asarray(scene.occupied_endpoints_xy_m).tolist(),
                        missing_rays=[dict(index=r.source_index, status=r.status,
                            origin=list(r.ray_origin_xy_m), direction=list(r.direction_xy))
                            for r in scene.rays if r.status != 'return'],
                        previous_steer_rad=previous, requested_steer_rad=planned[1],
                        body_length_m=owner.config.length_m, body_width_m=owner.config.width_m)
                    (A/'geometry632.json').write_text(json.dumps(geometry, indent=2))
        rows.append(row)
    return out


def main():
    global seq, adjacent
    digest = hashlib.sha256(TRACE.read_bytes()).hexdigest()
    spec = importlib.util.spec_from_file_location('r163_planner', A.parent/'adjacent_live_r152_v1/candidate.py')
    planner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(planner)
    planner.install()
    adjacent = planner.planner.bounded_adjacent_structure
    events = [json.loads(line) for line in TRACE.open()]
    LocalFollow._path_command, LocalFollow._partial_command = path, partial
    mismatches = []
    calls = 0
    try:
        owner, motion = make_controller(events[0]['data'])
        for event in events[1:-1]:
            seq = event['seq']
            if event['kind'] == 'reset':
                owner.reset()
            elif event['kind'] == 'call':
                out = freeze(replay_call(owner, event['data']['input'], motion))
                calls += 1
                if not equal_output(comparable_decision(out), comparable_decision(event['data']['output'])):
                    mismatches.append(seq)
    finally:
        LocalFollow._path_command, LocalFollow._partial_command = old_path, old_partial
    summary = dict(calls=calls, output_mismatches=mismatches,
                   rejected_calls=len(rows), positive_path_calls=sum(r['path_speed_mps'] > 0 for r in rows),
                   fully_observed_regions=sum('region' in r and r['region_block'] is None for r in rows),
                   reasons=dict(Counter(r['partial_reason'] for r in rows)),
                   trace_sha256=digest, trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest() == digest)
    (A/'diagnosis_geometry.json').write_text(json.dumps(dict(summary=summary, rows=rows), indent=2))
    print(json.dumps(summary))
    assert not mismatches and summary['trace_unchanged']


if __name__ == '__main__':
    main()
