"""관측된 옆차 GUARD의 현재 경로/이동 영역 재계산. 상태 전이·발행 없음."""
import importlib.util
from pathlib import Path
import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_follow import forward_planning_speed
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe

ROOT = Path(__file__).resolve().parents[4]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


previous = load('r170_previous', ROOT/'artifacts/validation/2026-11-12/adjacent_current_r151_v1/current.py')
geometry = previous.geometry


def supported(output):
    if not previous.supported(output):
        return False
    normal = output[2]['normal']
    sides = normal.get('selector_sides') or {}
    partial = normal.get('partial_observation') or {}
    return (set(sides) == {'left', 'right'}
            and all(all(type(s.get(k)) is bool for k in ('present', 'seen', 'passed'))
                    and (not s['present'] or s['seen']) for s in sides.values())
            and any(s['present'] for s in sides.values())
            and partial.get('allowed') is True and partial.get('reason') == 'fully_observed')


def rebase(output, submitted, current, scene, history, cfg):
    if not supported(output):
        return None
    normal = output[2]['normal']
    policy = ps.SelectorConfig()
    diagnostic = dict(rebased_selector_mode='GUARD', rebased_selector_choice='guard_baseline',
                      state_transition=False, source_permission_reused=False,
                      independent_guard_required=True, runtime_authorized=False)

    def stop(reason, **detail):
        return dict(speed_mps=0., steering_rad=current['previous_steering_rad'],
                    reason=reason, reference_s=current['reference_s'],
                    path_source_s=normal.get('path_source_s'), **diagnostic, **detail)

    if not cfg.guard_curve_trial or not cfg.traffic_recovery_trial:
        return stop('guard_current_trial_disabled')
    measured = current['speed_mps']
    planning_speed = forward_planning_speed(cfg, measured)
    if not geometry.finite(planning_speed) or planning_speed < 0:
        return stop('guard_current_invalid_speed')
    if output[0] > policy.guard_speed_cap_mps:
        return stop('guard_current_target_above_cap')
    geometric = (output[0], output[1], {**output[2], 'normal': {
        **normal, 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}})
    result = geometry.rebase(geometric, submitted, current, scene, history, cfg)
    if result is None:
        raise RuntimeError('guard geometry unexpectedly unsupported')
    if result['speed_mps'] <= 0:
        return {**result, **diagnostic}
    scan = MeasurementScan(**thaw(current['scan']['measurements']))
    if not previous.current_forward_coverage(scan, scene, history, cfg):
        return stop('guard_current_front_not_covered')
    lat = ps.lateral_config(cfg)
    view = ps.threat_view(scene, result['path_points'], lat)
    if view is None:
        return stop('guard_current_path_frame_missing')
    sides = normal['selector_sides']
    guarded = {name for name, _ in ps.SIDES
               if sides[name]['seen'] and not (sides[name]['passed'] and not sides[name]['present'])}
    if not guarded:
        return stop('guard_current_missing_guarded_side')
    if any(view[name]['present'] and name not in guarded for name, _ in ps.SIDES):
        return stop('guard_current_new_side')

    speed, target = planning_speed, result['speed_mps']
    steering, previous_steering = result['steering_rad'], current['previous_steering_rad']
    dt = current['dt_s']
    distance = ps.mx.envelope_distance(max(speed, target), scene.t_ref_s-scene.source_first_s,
                                      dt, ps.mx.ExecutionConfig())
    poses = np.vstack([np.vstack((ps.command_arc(a, max(distance, .01), lat),
                                  ps.extrema_arc(a, max(distance, .01), lat)))
                       for a in sorted({float(steering), float(previous_steering)})])
    approach, rejected = {}, []
    for name, sign in ps.SIDES:
        if name not in guarded:
            continue
        anchor = (normal['selector_guard_candidate'].get('guarded_sides', {}).get(name) or {}).get('anchor_m')
        if not geometry.finite(anchor):
            return stop('guard_current_anchor_missing')
        present = ps.side_extent(view['frame'], [[0., 0., 0.]], sign, lat)
        predicted = ps.side_extent(view['frame'], poses, sign, lat)
        approach[name] = dict(anchor_m=anchor, current_m=present, predicted_m=predicted)
        if predicted > anchor+policy.small_excursion_m and predicted > present+1e-9:
            rejected.append(name)
    context = {}
    region = ps.motion_region(scene, speed, target, steering, previous_steering, dt, cfg, lat, context=context)
    fringe = dict(accepted=False, reason='not_needed', unknown_filled=False)
    if region['unobserved']:
        fringe = bounded_rear_fringe(scene, scan, context['queries'][~context['free'] & ~context['masked']],
                                    context['padding'], cfg.length_m, cfg.width_m)
    blocked = ps.region_blocked(region, policy, bounded_unknown_risk=fringe['accepted'])
    detail = dict(guarded_sides=approach, region=region, rear_fringe=fringe,
                  measured_speed_mps=measured, forward_planning_speed_mps=planning_speed,
                  signed_history_preserved=True,
                  prediction_speed_mps=max(speed, target),
                  current_side_presence={name: view[name]['present'] for name, _ in ps.SIDES})
    if blocked:
        return stop('guard_current_'+blocked, **detail)
    if rejected:
        # motion_region은 전부 관측됐을 때 context를 생략한다. 동일 원 함수로 보충한다.
        if not context:
            _, queries = ps.motion_queries(scene, speed, target, steering, previous_steering, dt, cfg, lat)
            pad = ps.mx.ray_padding(scene, lat, cfg.object_motion_bound_mps)
            context = dict(queries=queries, free=ps.depth_support(scene, queries, pad))
        proof = ps.observed_guard_approach(view['frame'], context, approach, rejected)
        detail['current_approach'] = proof
        if not proof['accepted']:
            return stop('guard_current_approach_not_observed', **detail)
    return {**result, **diagnostic, **detail, 'reason': 'rebase_current_present_guard'}
