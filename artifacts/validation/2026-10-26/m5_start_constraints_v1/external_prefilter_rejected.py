"""관측 공간 안의 M3 측방 후보 선별 실험. 기본/ROS 운영 연결 없음.

이 후보는 모든 숨은 차량의 미래 진입을 배제하지 않는다. 현재 센서 지지·점유·기하·노면을 확인하는
조건부 비교이며 `runtime_authorized=False`다. 위험을 수용한 실차 제어/기본 채택으로 해석하지 않는다.
기존 v1의 시간 없는 상대 sweep을 쓰지 않고 현재 좌표에서 전진·회전을 포함한 실제 후보 외곽을 본다.
"""
import math
from dataclasses import replace
import numpy as np

from arena_autonomy.lateral_intent import (
    LateralIntentConfig, ObservationGrid, PathFrame, ego_state, cell_coordinates, corridor_reference,
    side_threats, offset_candidate, surface_check, lateral_clearance_check, committed_progress_m,
    rectangle_corners, wheel_contact_corners, GEOMETRY_EPS_M)
from arena_autonomy.observation_support import depth_support, source_valid, outside_body
from arena_autonomy.linear_observation import clip_halfplane


def path_observation_queries(scene, poses, cfg):
    """실제 차체 자세별 외곽 표본과 미관측 각도 세포의 교차 꼭짓점.

    2 cm 외곽 표본만으로 얇은 unknown 방향을 건너뛰지 않도록 해당 교차도 별도로 넣는다.
    현재 몸체 안쪽은 현재 점유 기하로 제외한다. 미래 숨은 차의 부재를 뜻하지 않는다.
    """
    lx, wy = cfg.length_m/2, cfg.width_m/2
    border = np.unique(np.vstack(([(x, y) for x in np.linspace(-lx, lx, 11) for y in (-wy, wy)],
                                  [(x, y) for x in (-lx, lx) for y in np.linspace(-wy, wy, 9)])), axis=0)
    unknown = [r for r in scene.rays if r.status != 'return']
    half = math.pi/500
    co, si = math.cos(half), math.sin(half)
    queries = []
    intersections = 0
    corners = rectangle_corners(poses, cfg.length_m, cfg.width_m)
    possible = np.zeros((len(poses), len(unknown)), dtype=bool)
    lowers, uppers = [], []
    for ray in unknown:
        dx, dy = ray.direction_xy
        lowers.append((co*dx+si*dy, -si*dx+co*dy))
        uppers.append((co*dx-si*dy, si*dx+co*dy))
    if unknown:
        origins = np.asarray([r.ray_origin_xy_m for r in unknown])
        directions = np.asarray([r.direction_xy for r in unknown])
        low, high = np.asarray(lowers), np.asarray(uppers)
        delta = corners[:, None, :, :]-origins[None, :, None, :]
        cross_low = low[None, :, None, 0]*delta[..., 1]-low[None, :, None, 1]*delta[..., 0]
        cross_high = high[None, :, None, 0]*delta[..., 1]-high[None, :, None, 1]*delta[..., 0]
        forward = delta[..., 0]*directions[None, :, None, 0]+delta[..., 1]*directions[None, :, None, 1]
        possible = ~(np.all(cross_low < 0, axis=2) | np.all(cross_high > 0, axis=2) | np.all(forward < 0, axis=2))
        # 전체 미래 차체가 unknown 부채꼴에 걸려도 교차가 현재 몸체 안에만
        # 있으면 최종 outside_body에는 아무것도 남지 않는다. 먼저 현재 몸체
        # 밖 네 반평면 조각과의 교차 가능성을 묶어 계산한다. 면적/길이로 작은
        # 조각을 제거하지 않으며, 최종 꼭짓점 생성은 아래의 원래 clipping이다.
        fragments, owners = [], []
        planes = (((lx, 0.), (0., 1.), -1), ((-lx, 0.), (0., 1.), 1),
                  ((0., wy), (1., 0.), 1), ((0., -wy), (1., 0.), -1))
        for index, polygon in enumerate(corners):
            for origin, direction, sign in planes:
                piece = clip_halfplane(polygon.tolist(), origin, direction, sign)
                if piece:
                    fragments.append(piece)
                    owners.append(index)
        external_possible = np.zeros_like(possible)
        if fragments:
            count = max(map(len, fragments))
            padded = np.array([p+[p[0]]*(count-len(p)) for p in fragments])
            for start in range(0, len(fragments), 32):
                delta = padded[start:start+32, None]-origins[None, :, None]
                cross_low = low[None, :, None, 0]*delta[..., 1]-low[None, :, None, 1]*delta[..., 0]
                cross_high = high[None, :, None, 0]*delta[..., 1]-high[None, :, None, 1]*delta[..., 0]
                # 사전 제외만 보수적으로 만든 수치 여유다. 최종 기하/관측의
                # 허용치는 바꾸지 않고 경계 근처는 원래 계산으로 보낸다.
                scale = np.maximum(1., np.max(np.abs(delta), axis=(2, 3)))
                scale *= np.maximum(1., np.max(np.abs(directions), axis=1))[None, :]
                tolerance = 128*np.finfo(float).eps*scale
                hit = ~(np.all(cross_low < -tolerance[..., None], axis=2) |
                        np.all(cross_high > tolerance[..., None], axis=2))
                np.logical_or.at(external_possible, owners[start:start+32], hit)
        possible &= external_possible
    for index, (pose, polygon) in enumerate(zip(poses, corners)):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world_border = border@np.array([[c, s], [-s, c]])+pose[:2]
        queries.extend(world_border)
        # 자세 사이 연속 외곽의 증명이 아니다. 시험 선별용 이산 외곽임을 명시한다.
        for j in np.flatnonzero(possible[index]):
            ray = unknown[j]
            ox, oy = ray.ray_origin_xy_m
            lower, upper = lowers[j], uppers[j]
            piece = clip_halfplane(clip_halfplane(polygon.tolist(), (ox, oy), lower, 1), (ox, oy), upper, -1)
            for exterior in outside_body(piece, cfg.length_m, cfg.width_m):
                queries.extend(exterior)
                queries.append(np.mean(exterior, axis=0))
                intersections += 1
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    outside = (abs(q[:, 0]) > lx+GEOMETRY_EPS_M) | (abs(q[:, 1]) > wy+GEOMETRY_EPS_M)
    return np.unique(q[outside], axis=0), intersections


def observed_candidate(scene, candidate, context, cfg, speed):
    """현재 센서에서만 후보를 선별한다. 기존 제동·HOLD·독립 보호의 대체가 아니다."""
    result = {'target_m': candidate['d_target_m'], 'eligible_for_trial': False, 'reasons': [],
              'hidden_future_risk_unresolved': True, 'continuous_sweep_certified': False}
    if not candidate['valid']:
        result['reasons'].append('geometry_invalid')
        return result
    surface = surface_check(context['frame'], candidate, cfg, current_pose=(0., 0., 0.))
    result['surface'] = {k: surface.get(k) for k in ('ok', 'grass_side', 'reason', 'edge_margin_m')}
    if not surface['ok']:
        result['reasons'].append(surface['reason'])
    if surface.get('grass_side'):
        lo, hi = surface['s_range_needing_edge']
        ref = corridor_reference(context['frame'], context['grid'].returns, lo-.05, hi+.05, cfg)
        if not ref[surface['grass_side']]['edge_verified']:
            result['reasons'].append('grass_edge_reference_unverified')
    distance = committed_progress_m(speed, context['age'], cfg)
    lateral = lateral_clearance_check(candidate, context['grid'].returns, context['grid'].ages,
                                      context['frame'], context['corridor'], cfg, distance)
    result['lateral_clearance'] = lateral
    if not lateral['ok']:
        result['reasons'].append('lateral_clearance_short')
    keep = candidate['progress'] <= distance+GEOMETRY_EPS_M
    poses = candidate['poses'][keep]
    if candidate['progress'][-1] < distance-GEOMETRY_EPS_M:
        result['reasons'].append('stopping_horizon_unavailable')
    q, thin = path_observation_queries(scene, poses, cfg)
    # 노면 접지 길이와 달리 충돌 외곽에는 바퀴 지름 전체를 쓴다.
    wheel_cfg = replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m)
    wheel_q = wheel_contact_corners(poses, candidate['steering_rad'][keep], wheel_cfg).reshape(-1, 2)
    wheel_q = wheel_q[(abs(wheel_q[:, 0]) > cfg.length_m/2+GEOMETRY_EPS_M) |
                      (abs(wheel_q[:, 1]) > cfg.width_m/2+GEOMETRY_EPS_M)]
    q = np.unique(np.vstack((q, wheel_q)), axis=0)
    # 기존 G1 점유의 고정 여유 25 mm + 거리 30 mm + 취득 시차 상계 0.5 m/s를 유지한다.
    pad = np.array([.025+cfg.budget.range_bias_m+.5*(scene.t_ref_s-r.acquired_at_s)
                    for r in scene.rays])
    support = depth_support(scene, q, pad)
    result.update(observation_queries=len(q), unsupported_queries=int(np.sum(~support)),
                  unknown_sector_intersections=thin, required_progress_m=distance)
    if not np.all(support):
        result['reasons'].append('current_sweep_unobserved')
    result['eligible_for_trial'] = not result['reasons']
    return result


class ObservedLateralTrial:
    """저장 입력용 선별 상태. 운영 미연결이며 runtime_authorized는 항상 false다."""

    def __init__(self, cfg=None):
        self.cfg = cfg or LateralIntentConfig()
        self.reset()

    def reset(self):
        self.active = False
        self.clear_fresh = 0
        self.epoch = self.last_ref = self.last_source = None

    def step(self, scene, path, speed):
        out = {'runtime_authorized': False, 'hidden_future_risk_unresolved': True,
               'mode': 'OFF', 'selected': None, 'checks': {}, 'path': path}
        cfg = self.cfg
        if not cfg.enabled:
            return out
        if (not math.isfinite(speed) or speed < 0 or not source_valid(scene)):
            self.reset()
            return {**out, 'mode': 'HOLD_INPUT', 'path': None}
        if (self.epoch != scene.source_epoch or (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                (self.last_source is not None and scene.source_first_s < self.last_source)):
            self.reset()
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        self.epoch, self.last_ref, self.last_source = scene.source_epoch, scene.t_ref_s, scene.source_first_s
        if path is None:
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        try:
            frame = PathFrame(path)
        except (ValueError, TypeError):
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        state = ego_state(frame, cfg)
        grid = ObservationGrid(scene, cfg)
        corridor = corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m, state['s0']+cfg.horizon_m, cfg)
        s, l = cell_coordinates(grid, frame)
        threats = side_threats(frame, state, grid, cfg, s, l, grid.observed_free())
        present = [name for name in ('left', 'right') if threats[name]['present']]
        out.update(state=state, threats=threats)
        if not self.active and not present:
            return {**out, 'mode': 'BASELINE', 'selected': 'baseline'}
        self.active = True
        self.clear_fresh = 0 if present else self.clear_fresh+int(fresh)
        context = {'frame': frame, 'grid': grid, 'corridor': corridor, 'age': scene.t_ref_s-scene.source_first_s}
        hold = offset_candidate(frame, state, state['d0'], cfg, 'hold')
        candidates = {'hold': hold}
        order = []
        if len(present) == 1:
            side = present[0]
            sign = 1. if side == 'left' else -1.
            gap = cfg.budget.lateral_clearance(True, hold['curved'], context['age'])[0]
            edge = threats[side]['nearest']['near_edge_l_m']
            target = sign*min(sign*state['d0'], sign*edge-state['half_span_m']-gap)
            limit = cfg.policy_limit_m(hold['curved'])['selected_m']
            target = float(np.clip(target, -limit, limit))
            if sign*(target-state['d0']) < -.005:
                candidates['yield'] = offset_candidate(frame, state, target, cfg, 'yield')
                order.append('yield')
        elif not present and self.clear_fresh >= cfg.release_fresh:
            candidates['center'] = offset_candidate(frame, state, 0., cfg, 'center')
            order.append('center')
        order.append('hold')
        checks = {name: observed_candidate(scene, c, context, cfg, speed) for name, c in candidates.items()}
        out['checks'] = checks
        for key in order:
            if checks[key]['eligible_for_trial']:
                if key == 'center' and abs(state['d0']) < .005:
                    self.active = False
                return {**out, 'mode': key.upper()+'_TRIAL', 'selected': key, 'path': candidates[key]['path']}
        return {**out, 'mode': 'HOLD_STOP_TRIAL', 'path': None}
