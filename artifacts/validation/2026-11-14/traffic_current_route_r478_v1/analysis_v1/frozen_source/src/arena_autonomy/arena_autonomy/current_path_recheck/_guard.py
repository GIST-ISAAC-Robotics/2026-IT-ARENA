"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.

원 함수: 2026-11-12/adjacent_current_r151_v1/current.py
이관/동등성 근거: current_recheck_module_r176_v1.
"""
import math
import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.known_structure import classify, parse_sectors
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from arena_autonomy.scan_input import audit_scan_front
from arena_autonomy.local_follow import forward_planning_speed
from . import _geometry as geometry
from . import _gaps as body
from . import _gaps as adjacent


def supported(output):
    normal = output[2].get('normal') or {}
    guard = normal.get('selector_guard_candidate') or {}
    return (geometry.finite(output[0]) and output[0] > 0
            and output[2].get('handoff_state') == 'FOLLOW'
            and normal.get('reason') == 'local_follow'
            and normal.get('selector_mode') == 'GUARD'
            and normal.get('selector_choice') == 'guard_baseline'
            and guard.get('accepted') is True)


def current_forward_coverage(scan, scene, history, cfg):
    """현재 전방 광선 분류만 확인한다. 부분 관측의 이력/래치 허가가 아니다."""
    if audit_scan_front(scan, history.config.lidar_yaw_rad, metadata_only=True).reason != 'ok':
        return False
    excused = np.zeros(len(scene.rays), dtype=bool)
    if cfg.known_structure_policy != 'off':
        _, excused, _ = classify(scan, parse_sectors(cfg.known_structure_sectors_json))
    return all(ray.status == 'return' or excused[ray.source_index]
               for ray in scene.rays
               if abs(math.atan2(ray.direction_xy[1], ray.direction_xy[0])) <= math.radians(100))


def rebase(output, submitted, current, scan, scene, history, cfg):
    if not supported(output):
        return None
    normal = output[2]['normal']; original_guard = normal['selector_guard_candidate']
    diagnostic = dict(rebased_selector_mode='GUARD', rebased_selector_choice='guard_baseline',
                      state_transition=False, source_permission_reused=False,
                      independent_guard_required=True, runtime_authorized=False)

    def stop(reason, **detail):
        return dict(speed_mps=0., steering_rad=current['previous_steering_rad'],
                    reason=reason, reference_s=current['reference_s'],
                    path_source_s=normal.get('path_source_s'), **diagnostic, **detail)

    sides = normal.get('selector_sides') or {}
    if set(sides) != {'left','right'}:
        return stop('guard_rebase_missing_side_state')
    if any(side.get('present') is not False for side in sides.values()):
        return stop('guard_rebase_present_side_outside_scope')
    guarded = [name for name, side in sides.items()
               if side.get('seen') is True and side.get('passed') is False]
    if not guarded or not cfg.guard_curve_trial or not cfg.traffic_recovery_trial:
        return stop('guard_rebase_original_state_invalid')
    # 계획기와 같은 전진 정책을 사용한다. 원 부호 있는 운동 이력/입력은 그대로다.
    measured = current['speed_mps']
    planning_speed = forward_planning_speed(cfg, measured)
    if not geometry.finite(planning_speed) or not 0 <= planning_speed <= .35 or output[0] > .35:
        return stop('guard_rebase_speed_outside_scope')
    partial = normal.get('partial_observation') or {}
    if partial.get('allowed') is not True or partial.get('reason') != 'fully_observed':
        return stop('guard_rebase_original_partial_outside_scope')
    # 순수 기하 helper의 진입 분류만 맞춘다. 결과를 즉시 반환하지 않고 아래
    # 현재 GUARD 검사를 모두 수행한다. 실제 선택기 상태를 SOLO로 바꾸지 않는다.
    geometric_input = (output[0], output[1], {**output[2], 'normal': {
        **normal, 'selector_mode':'SOLO', 'selector_choice':'baseline'}})
    result = geometry.rebase(geometric_input, submitted, current, scene, history, cfg)
    if result is None:
        raise RuntimeError('guard geometry unexpectedly unsupported')
    if result['speed_mps'] <= 0:
        return {**result, **diagnostic}
    if not current_forward_coverage(scan, scene, history, cfg):
        return stop('guard_rebase_current_front_not_covered')
    lat = ps.lateral_config(cfg)
    view = ps.threat_view(scene, result['path_points'], lat)
    if view is None:
        return stop('guard_rebase_current_path_frame_missing')
    if any(view[name]['present'] for name, _ in ps.SIDES):
        return stop('guard_rebase_current_side_present')
    speed = planning_speed; target = result['speed_mps']
    previous = current['previous_steering_rad']; steering = result['steering_rad']
    dt = current['dt_s']; age = scene.t_ref_s-scene.source_first_s
    distance = ps.mx.envelope_distance(max(speed,target), age, dt, ps.mx.ExecutionConfig())
    poses = np.vstack([np.vstack((ps.command_arc(angle,max(distance,.01),lat),
                                  ps.extrema_arc(angle,max(distance,.01),lat)))
                       for angle in sorted({float(steering),float(previous)})])
    approach, rejected = {}, []
    policy = ps.SelectorConfig()
    for name, sign in ps.SIDES:
        if name not in guarded:
            continue
        old = (original_guard.get('guarded_sides') or {}).get(name) or {}
        anchor = old.get('anchor_m')
        if not geometry.finite(anchor):
            return stop('guard_rebase_original_anchor_missing')
        present_extent = ps.side_extent(view['frame'], [[0.,0.,0.]], sign, lat)
        predicted = ps.side_extent(view['frame'], poses, sign, lat)
        approach[name] = dict(anchor_m=anchor, current_m=present_extent, predicted_m=predicted)
        if predicted > anchor+policy.small_excursion_m and predicted > present_extent+1e-9:
            if cfg.guard_observation_policy != 'selected_r181':
                return stop('guard_rebase_toward_unreleased_side', guarded_sides=approach)
            rejected.append(name)
    context = {}
    region = ps.motion_region(scene,speed,target,steering,previous,dt,cfg,lat,context=context)
    proof = dict(accepted=False, reason='not_needed', unknown_filled=False)
    if region['unobserved']:
        queries = context['queries'][~context['free'] & ~context['masked']]
        proof = bounded_rear_fringe(scene,scan,queries,context['padding'],cfg.length_m,cfg.width_m)
        original_proof = original_guard.get('same_path_gap') or {}
        if (not proof['accepted'] and original_proof.get('accepted') is True
                and original_proof.get('policy') == 'guard_low_speed_body_nonreturn_r136'):
            proof = body.bounded_body_nonreturn(scene,scan,queries,context['padding'],
                                               cfg.length_m,cfg.width_m,speed,target)
        elif (not proof['accepted'] and original_proof.get('accepted') is True
                and original_proof.get('policy') == 'guard_adjacent_structure_gap_r149'
                and cfg.known_structure_policy == 'structure_aware_v1'):
            # 이전 허가를 복사하지 않는다. 현재 광선·거리·장착 마스크와
            # 현재 실제속도로 같은 좁은 위험 분류를 처음부터 검사한다.
            proof = adjacent.bounded_adjacent_structure(scene,scan,queries,context['padding'],
                cfg.length_m,cfg.width_m,speed,target,parse_sectors(cfg.known_structure_sectors_json))
    blocked = ps.region_blocked(region,policy,bounded_unknown_risk=proof['accepted'])
    if blocked:
        if rejected:
            return stop('guard_rebase_toward_unreleased_side', region=region,
                        current_gap_proof=proof, guarded_sides=approach, current_region_blocked=blocked)
        return stop('guard_rebase_'+blocked, region=region, current_gap_proof=proof)
    approach_detail = {}
    if rejected:
        if not context:
            _, queries = ps.motion_queries(scene,speed,target,steering,previous,dt,cfg,lat)
            pad = ps.mx.ray_padding(scene,lat,cfg.object_motion_bound_mps)
            context = dict(queries=queries,free=ps.depth_support(scene,queries,pad))
        observed = ps.observed_guard_approach(view['frame'],context,approach,rejected,
            region=region,selector_config=policy,length_m=cfg.length_m,width_m=cfg.width_m)
        approach_detail['current_approach'] = observed
        if not observed['accepted']:
            return stop('guard_rebase_toward_unreleased_side',guarded_sides=approach,
                        region=region,current_gap_proof=proof,**approach_detail)
    return {**result, **diagnostic, **approach_detail, 'reason':'rebase_current_guard',
            'guarded_sides':approach, 'region':region, 'current_gap_proof':proof,
            'current_side_presence':{name:view[name]['present'] for name,_ in ps.SIDES},
            'forward_coverage_only':True}
