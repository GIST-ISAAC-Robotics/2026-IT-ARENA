"""외부 r386 동결 geometry와 r377 세 입력만 비교. 발행/ROS/장치 없음."""
import argparse
import copy
from dataclasses import asdict, replace
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SOURCE = Path(os.environ['R386_SOURCE_ROOT']).resolve()
INPUT = Path(os.environ['R386_INPUT_ROOT']).resolve()
R386 = SOURCE.parent/'r386'
sys.path.insert(0, str(R386))
from support import load, verify


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
    return result


def inputs(compare):
    path = INPUT/'artifacts/validation/2026-11-13/released_cad_drive_r377_v1/run1/follow_decisions.jsonl'
    index = -1; result = []
    for line in path.open(encoding='utf8'):
        event = json.loads(line)
        if event['kind'] == 'header': header = event['data']
        if event['kind'] != 'call' or 'current_rebase' not in event['data']: continue
        index += 1
        if index not in (123, 125, 222): continue
        current = event['data']['current_rebase']; plan = current['compute']['plan']
        result.append(dict(index=index, event_seq=event['seq'], header=header,
            output=plan['output'], submitted=compare.plan_reference(plan['input']), current=current['input']))
    assert index == 222 and [r['event_seq'] for r in result] == [606, 610, 804]
    return result


def geometry_record(scene, context, current, steering, region, cfg):
    """실제 최종 union 질의와 각 조향의 정확한 소유자. 별도 진단이며 비용 표본 아님."""
    from arena_autonomy import pass_release_trial as ps
    from arena_autonomy.point_set import unique_xy
    from arena_autonomy.rear_fringe_trial import _query_brackets
    lat = ps.lateral_config(cfg); members = {}; poses_by_angle = {}
    named = dict(current=float(steering), previous=float(current['previous_steering_rad']))
    for angle in sorted(set(named.values())):
        poses = ps.extrema_arc(angle, max(region['distance_m'], .01), lat)
        q, _ = ps.mx.body_queries(scene, poses, np.full(len(poses), angle), lat)
        q = unique_xy(q.reshape(-1, 2))
        q = q[(np.abs(q[:, 0]) > cfg.length_m/2+1e-9) | (np.abs(q[:, 1]) > cfg.width_m/2+1e-9)]
        members[angle] = {tuple(point) for point in q}
        poses_by_angle[str(angle)] = poses.tolist()
    union = set().union(*members.values())
    actual = {tuple(point) for point in context['queries']}
    assert union == actual, 'per-steering reconstruction differs from actual final queries'
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    queries = []
    brackets = _query_brackets(context['queries'], origins, directions)
    for i, (point, (_, angles, lo, hi)) in enumerate(zip(context['queries'], brackets)):
        queries.append(dict(xy_m=point.tolist(),
            owners=[name for name, angle in named.items() if tuple(point) in members[angle]],
            free=bool(context['free'][i]), masked=bool(context['masked'][i]),
            unknown=bool(not context['free'][i] and not context['masked'][i]),
            bracketing_source_indices=[int(lo), int(hi)],
            bracket_offsets_rad=[float(angles[lo]), float(angles[hi])]))
    return dict(region=region, steering_rad=named, poses_by_steering_rad=poses_by_angle,
                per_steering_union_exact=True, queries=queries)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); destination = args.output.resolve()
    assert destination.is_relative_to(HERE) and not destination.exists()
    destination.mkdir(); verify(); compare, imports = load('native_v1')
    from arena_autonomy import pass_release_trial as ps
    from arena_autonomy.local_scene import blind_spec
    from arena_autonomy.vehicle_geometry import planning_parameters, preview_parameters
    from range_mask_component import bounded_adjacent_structure as component
    budget_candidate = module('r396_original_budget_candidate', R386/'candidate.py')
    selected = yaml.safe_load((SOURCE/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml').read_bytes())['vehicle']
    planning, sender = planning_parameters(selected), preview_parameters(selected)
    expected = {r['index']: r for r in json.loads((HERE/'expected_final_three.json').read_bytes())}
    rows = []; error = None
    try:
        for record in inputs(compare):
            original = copy.deepcopy(record); index = record['index']; current = record['current']
            cfg = compare.FollowConfig(**record['header']['follow_config'])
            assert (cfg.length_m, cfg.width_m, cfg.wheelbase_m, cfg.max_steering_rad) == (
                planning['vehicle_length_m'], planning['vehicle_width_m'], sender['wheelbase_m'], sender['max_steering_angle_rad'])
            cfg = replace(cfg, track_m=planning['vehicle_track_width_m'],
                wheel_radius_m=planning['vehicle_wheel_radius_m'], wheel_width_m=planning['vehicle_wheel_width_m'])
            info, raw, history = compare.restore_call(current, compare.MotionConfig(**record['header']['motion_config']))
            scene = compare.motion_scene(raw, history, info['reference_s'], timing_verified=info['timing_verified'],
                epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
            profile = info['rear_half_angle_deg']; sectors = blind_spec(profile)[1]
            call = (record['output'], record['submitted'], current, scene, history, cfg)
            start = time.perf_counter_ns()
            baseline = compare.policy.rebase(*call, require_current_region=True)
            baseline_ms = (time.perf_counter_ns()-start)/1e6
            assert compare.freeze(baseline) == expected[index]['before'], 'frozen final baseline mismatch'
            normal = record['output'][2]['normal']
            assert baseline['speed_mps'] == 0 and normal['selector_mode'] == 'SOLO' and normal['selector_choice'] == 'baseline'
            assert 0 <= current['speed_mps'] <= .15 and 0 < current['dt_s'] <= .1

            def prove(s, scan, q, pad, length, width, speed, target, **unused):
                return component(s, scan, q, pad, length, width, speed, target, sectors,
                    mask_profile=profile, motion_epoch=info['epoch'], timing_verified=info['timing_verified'])

            # helper-only: 원 baseline의 .10 relative retry와 같은 core/질의/목표를 쓴다.
            start = time.perf_counter_ns(); core = compare.policy.core_policy.rebase(
                (.10, record['output'][1], record['output'][2]), *call[1:])
            assert core and core['reason'] == 'rebase_current_follow' and 0 < core['speed_mps'] <= .10
            assert all(core.get(k) == baseline.get(k) for k in (
                'steering_rad', 'reference_s', 'path_source_s', 'original_reference_s', 'path_points'))
            context = {}; region = ps.motion_region(scene, current['speed_mps'], core['speed_mps'],
                core['steering_rad'], current['previous_steering_rad'], current['dt_s'], cfg,
                ps.lateral_config(cfg), context=context)
            assert region == baseline['current_relative_body_retry']['region']
            q = context['queries'][~context['free'] & ~context['masked']]
            proof = prove(scene, raw, q, context['padding'], cfg.length_m, cfg.width_m,
                          current['speed_mps'], core['speed_mps'])
            blocked = ps.region_blocked(region, ps.SelectorConfig(), bounded_unknown_risk=proof['accepted'])
            accepted = bool(region['queries'] and proof['accepted'] and blocked is None)
            helper = dict(applied=accepted, target_mps=core['speed_mps'], region=region, proof=proof, blocked=blocked)
            helper_result = baseline if not accepted else {
                **core, 'current_route_proof': {**baseline['current_route_proof'], 'accepted': True,
                                              'region': region, 'bounded_current_risk': helper}}
            helper_ms = (time.perf_counter_ns()-start)/1e6
            geometry = dict(relative_010=geometry_record(scene, context, current, core['steering_rad'], region, cfg))

            # 원 r386 파일 불변. 이 호출에서만 gap 함수와 진단 수집을 교체한다.
            old_proof, old_ps = budget_candidate.bounded_body_nonreturn, budget_candidate.ps
            captured = []
            def capture(*a, **kw):
                result = ps.motion_region(*a, **kw)
                captured.append((a[3], result, dict(kw['context'])))
                return result
            costs = {}; start = time.perf_counter_ns()
            try:
                budget_candidate.bounded_body_nonreturn = prove
                budget_candidate.ps = SimpleNamespace(**{**vars(ps), 'motion_region': capture})
                combined = budget_candidate.try_from_current_result(baseline, *call,
                    core_rebase=compare.policy.core_policy.rebase, require_current_region=True, costs=costs)
            finally:
                budget_candidate.bounded_body_nonreturn = old_proof; budget_candidate.ps = old_ps
            combined_ms = (time.perf_counter_ns()-start)/1e6
            assert len(captured) <= 1, 'more than one final region check'
            for steer, final_region, final_context in captured:
                geometry['distance_plus_component'] = geometry_record(scene, final_context, current, steer, final_region, cfg)
            diagnostic = combined.get(budget_candidate.DIAGNOSTIC) or {}
            if diagnostic.get('applied'):
                assert diagnostic['final_full_checks'] == diagnostic['additional_target_attempts'] == 1
                assert diagnostic['region']['distance_m'] <= diagnostic['proposal']['allowed_distance_m']
                assert diagnostic['proof']['accepted'] and not diagnostic['unknown_filled']
            else:
                assert compare.freeze({k:v for k,v in combined.items() if k != budget_candidate.DIAGNOSTIC}) == compare.freeze(baseline)
            assert record == original, 'input changed'
            row = dict(index=index, event_seq=record['event_seq'], config=asdict(cfg), baseline=baseline,
                baseline_exact=True, helper_only=helper_result, helper_diagnostic=helper,
                distance_plus_component=combined, original_r386=expected[index]['after'],
                original_r386_adopted=False, runtime_authorized=False,
                diagnostic_wall_ms=dict(baseline=baseline_ms, helper_only=helper_ms,
                                        distance_plus_component=combined_ms), combined_stages_ms=costs)
            rows.append(row)
            ray_record = [dict(source_index=r.source_index, source_angle_rad=raw.angles_rad[i],
                raw_range_m=raw.ranges_m[i], status=r.status, origin_xy_m=r.ray_origin_xy_m,
                direction_xy=r.direction_xy, hit_xy_m=r.hit_xy_m, acquired_at_s=r.acquired_at_s)
                for i,r in enumerate(scene.rays)]
            (destination/f'queries_{index}.json').write_text(json.dumps(compare.freeze(dict(
                index=index, reference_s=scene.t_ref_s, source_epoch=scene.source_epoch,
                mask_profile=profile, rays=ray_record, arms=geometry)), ensure_ascii=False, indent=2)+'\n', encoding='utf8')
            (destination/f'row_{index}.json').write_text(json.dumps(compare.freeze(row), ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    except Exception as exc:
        error = repr(exc)
    source_preserved = True
    try:
        verify()
    except Exception as exc:
        source_preserved = False
        error = (error+'; ' if error else '')+repr(exc)
    helper_recovery = [r['index'] for r in rows if r['baseline']['speed_mps'] == 0 < r['helper_only']['speed_mps']]
    combined_recovery = [r['index'] for r in rows if r['baseline']['speed_mps'] == 0 < r['distance_plus_component']['speed_mps']]
    report = dict(passed=error is None and len(rows)==3, error=error, completed=len(rows), imports=imports,
        helper_only_recovery=helper_recovery, distance_plus_component_recovery=combined_recovery,
        discard_if_no_recovery=not helper_recovery and not combined_recovery,
        independent_arms=True, new_default=False, drive_executed=False, r386_adopted=False,
        timing_scope='single diagnostic calls; no performance benchmark', source_preserved=source_preserved)
    (destination/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, ensure_ascii=False))
    if not report['passed']: raise SystemExit(1)


if __name__ == '__main__': main()
