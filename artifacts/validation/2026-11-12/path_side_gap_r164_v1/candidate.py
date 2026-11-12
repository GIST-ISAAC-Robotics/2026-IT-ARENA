"""저속 GUARD의 차체 옆 좁은 결측 후보. 거리 반환/자유 공간을 만들지 않는다.

고정 60 cm 이웃/전방 각도 경계 대신 현재 이동 영역+기존 나이별 점유 여유를
사용한다. 기본 코드에는 설치되지 않으며 r152의 명시 후보를 잇는 저장 비교용이다.
"""
import importlib.util
import math
from pathlib import Path
import numpy as np

import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.scan_input import scan_first_stamp

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r164_prior', A.parent/'adjacent_live_r152_v1/candidate.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
REASON = 'bounded_current_body_side_gap'
old_path = old_partial = old_same = old_reasons = None


def body_gap(scene, scan, queries, padding, length, width, speed, target):
    result = dict(accepted=False, reason='outside_scope', unknown_filled=False,
        residual_hidden_object_risk=True, maximum_exterior_m=.01,
        maximum_gap_rad=math.radians(6), maximum_local_chord_m=.05,
        policy='current_body_side_gap_r164')
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = np.asarray(padding, dtype=float)
    if (scan is None or not source_valid(scene) or not all(math.isfinite(v) for v in (length, width, speed, target))
            or not 0 <= speed <= .15 or not 0 < target <= .15 or length <= 0 or width <= 0
            or not len(q) or not np.all(np.isfinite(q)) or pad.shape != (len(scene.rays),)
            or not np.all(np.isfinite(pad)) or np.any(pad < 0)):
        return result
    exterior = np.abs(q[:, 1])-width/2
    result['exterior_max_m'] = float(np.max(exterior))
    if np.any(abs(q[:, 0]) > length/2) or np.any(exterior <= 0) or np.any(exterior > .01):
        return result
    # 같은 현 입력의 모든 광선 원점과 실제 질의 반경에 점유 여유를 더한다.
    # 이웃 점을 멀리 있다고 간주하거나 실제속도를 목표속도로 대신하지 않는다.
    origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
    radius = np.linalg.norm(q[:, None, :]-origins[None, :, :], axis=2)
    result['minimum_neighbor_range_m'] = float(np.max(radius+pad[None, :]))
    result['neighbor_basis'] = 'current_query_radius_plus_existing_per_ray_padding'
    checked = _bounded_nonreturn(scene, scan, q, pad, result)
    if not checked['accepted']:
        return checked
    if len(checked['gaps']) != 1:
        return {**checked, 'accepted': False, 'reason': 'multiple_components'}
    gap = checked['gaps'][0]
    member = set(range(gap['first'], gap['last']+1))
    # 전역 100도 경계가 자르는 바로 그 성분만 대상이다. 추가 전방 결측은 허용하지 않는다.
    directions = np.asarray([math.atan2(r.direction_xy[1], r.direction_xy[0]) for r in scene.rays])
    component_angles = np.abs(directions[sorted(member)])
    if not (np.min(component_angles) <= math.radians(100) < np.max(component_angles)):
        return {**checked, 'accepted': False, 'reason': 'not_front_edge_component'}
    front_unknown = {i for i, r in enumerate(scene.rays)
                     if r.status == 'unknown' and abs(directions[i]) <= math.radians(100)}
    if not front_unknown or not front_unknown.issubset(member):
        return {**checked, 'accepted': False, 'reason': 'other_front_missing'}
    chord = 2*result['minimum_neighbor_range_m']*math.sin(gap['gap_rad']/2)
    if chord > .05:
        return {**checked, 'accepted': False, 'reason': 'wide_local_chord', 'local_chord_m': chord}
    return {**checked, 'reason': REASON, 'local_chord_m': chord}


def history_check(policy, scene, history, queries, points):
    result = dict(accepted=False, reason='history_unavailable')
    if policy.latched:
        return {**result, 'reason': 'known_latch_preserved'}
    previous = policy.previous
    if previous is None or not 0 <= scene.t_ref_s-previous[1] <= .3:
        return result
    old = transport_points(previous[0], history, previous[1], scene.t_ref_s)
    current = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(current):
        return result
    # 기존 2 m/s 상대 이동 가정과 3 cm 여유를 유지한다. 재관측 안 된 옛 점이
    # 이번 좁은 띠로 접근할 수 있으면 진행하지 않는다. 나이 만료로 지우지 않는다.
    reach = .03+2*(scene.t_ref_s-previous[1])
    near = old[np.min(np.linalg.norm(old[:, None, :]-queries[None, :, :], axis=2), axis=1) <= reach] if len(old) else old
    missing = [p.tolist() for p in near if np.min(np.linalg.norm(current-p, axis=1)) > .02]
    return dict(accepted=not missing, reason='current_reobserved' if not missing else 'previous_point_missing',
                previous_source_s=previous[2], relative_motion_reach_m=reach,
                relevant_points=len(near), missing_points=missing)


def path(owner, *args, **kwargs):
    result = old_path(owner, *args, **kwargs)
    owner._r164_path = (result, owner.held, owner.release_count, owner.hold_reason)
    return result


def partial(owner, scene, history, speed, side, previous=0., dt=.02, raw_scan=None):
    owner._r164_path = None
    result = old_partial(owner, scene, history, speed, side, previous, dt, raw_scan)
    proof = result[2].get('partial_observation') or {}
    stored = owner._r164_path
    selector = owner.pass_selector
    if (stored is None or selector is None or selector.mode != 'GUARD'
            or not owner.config.traffic_recovery_trial or not 0 <= speed <= .15
            or proof.get('allowed') is not False or proof.get('reason') != 'unbounded_missing'
            or result[2].get('reason') != 'follow_unbounded_missing'):
        return result
    planned, held, releases, hold_reason = stored
    b = planned[2]
    if (b.get('reason') not in ('local_follow', 'follow_release_wait')
            or b.get('boundary_from_memory') is not False or b.get('path_source_s') != scene.source_first_s
            or b.get('target_speed_before_hold_mps', 0) <= 0):
        return result
    context = {}
    region = ps.motion_region(scene, speed, .15, planned[1], previous, dt,
                              owner.config, selector.lat, context=context)
    if not context or not region['unobserved']:
        return result
    q = context['queries'][~context['free'] & ~context['masked']]
    checked = body_gap(scene, raw_scan, q, context['padding'], owner.config.length_m,
                       owner.config.width_m, speed, .15)
    remembered = history_check(owner.partial_policy, scene, history, q, scene.occupied_endpoints_xy_m)
    detail = {**result[2], 'body_side_gap_candidate': dict(geometry=checked, history=remembered)}
    if not checked['accepted'] or not remembered['accepted']:
        return result[0], result[1], detail
    # 실제 구조 그림자/점유 검사의 0 상한도 유지한다.
    structure = proof.get('known_structure') or {}
    cap = structure.get('speed_cap_mps')
    if cap is not None and (not math.isfinite(cap) or cap <= 0):
        return result[0], result[1], detail
    target = min(.15, planned[0], cap if cap is not None else math.inf)
    source = scan_first_stamp(raw_scan)
    owner.partial_policy._remember(scene.occupied_endpoints_xy_m, scene.t_ref_s, source,
                                  source != owner.partial_policy.last_source)
    # 정상 경로의 서로 다른 관측 확인 횟수는 유지한다. 이번 전역 거절만 되돌린다.
    owner.held, owner.release_count, owner.hold_reason = held, releases, hold_reason
    partial_proof = {**proof, 'allowed': True, 'reason': REASON, 'speed_cap_mps': min(.15, cap) if cap is not None else .15,
                     'unknown_filled': False, 'body_side_gap': checked}
    return target, planned[1], {**detail, **b, 'partial_observation': partial_proof}


def same_path(owner, scene, scan, region, context, speed, target, steer, previous, baseline):
    partial_proof = baseline.get('partial_observation') or {}
    if (partial_proof.get('allowed') is True and partial_proof.get('reason') == REASON
            and context and region['unobserved']):
        q = context['queries'][~context['free'] & ~context['masked']]
        return body_gap(scene, scan, q, context['padding'], owner.follow_config.length_m,
                        owner.follow_config.width_m, speed, target)
    return old_same(owner, scene, scan, region, context, speed, target, steer, previous, baseline)


def install():
    global old_path, old_partial, old_same, old_reasons
    if LocalFollow._partial_command is partial:
        return
    prior.install()
    old_path, old_partial, old_same = LocalFollow._path_command, LocalFollow._partial_command, ps.same_path_risk
    old_reasons = ps.GUARD_PARTIAL_REASONS
    LocalFollow._path_command, LocalFollow._partial_command = path, partial
    ps.same_path_risk = same_path
    ps.GUARD_PARTIAL_REASONS = (*old_reasons, REASON)


def uninstall():
    LocalFollow._path_command, LocalFollow._partial_command = old_path, old_partial
    ps.same_path_risk, ps.GUARD_PARTIAL_REASONS = old_same, old_reasons
