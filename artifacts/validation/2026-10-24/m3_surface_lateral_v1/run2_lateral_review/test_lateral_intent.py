"""측방 의도 제한기(M3 후보 v1)의 합성 반례. 운영 노드·Gazebo·정답 지면을 쓰지 않는다."""
import math

import numpy as np
import pytest

from arena_autonomy.lateral_intent import (
    LateralBudget, LateralIntentConfig, LateralIntentLimiter, ObservationGrid, PathFrame, EGO, FREE, UNOBS,
    bypass_candidate, corridor_reference, ego_state, offset_candidate, steering_protrusion_m, surface_check,
    verify_candidate, hidden_reach, corridor_mask, cell_coordinates)
from arena_autonomy.local_scene import scene_from_scan

GRASS = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass')
ROAD = LateralIntentConfig(enabled=True, surface_policy='road_only')


def box(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def cast(polygons, n=500, sensor=(.06, 0.), dropout=None, t_ref=1., max_range=12.):
    """정답이 아닌 합성 장면의 광선 투사. base_link 고정 센서(운동 없음)."""
    angles = -math.pi+np.arange(n)*2*math.pi/n
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    o = np.asarray(sensor)
    best = np.full(n, np.inf)
    for poly in polygons:
        a, b = poly, np.roll(poly, -1, axis=0)
        e = b-a
        for k in range(len(a)):
            denom = d[:, 0]*e[k, 1]-d[:, 1]*e[k, 0]
            w = a[k]-o
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (w[0]*e[k, 1]-w[1]*e[k, 0])/denom
                u = (w[0]*d[:, 1]-w[1]*d[:, 0])/denom
            ok = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0) & (u <= 1)
            best = np.where(ok & (t < best), t, best)
    ranges = np.where(best <= max_range, best, np.inf)
    if dropout is not None:
        deg = np.degrees(angles)
        ranges = np.where(dropout(deg), np.nan, ranges)
    stamps = t_ref-.1+np.arange(n)*(.1/n)
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=2*math.pi/n, range_min=.05,
                           range_max=max_range, acquired_at_s=stamps.tolist(), t_ref_s=t_ref)


def straight(d0, extra=(), wall_gap_right=None, right_wall_offset=.425, dropout=None):
    """자차가 도로 중심에서 d0만큼 옆에 있는 직선. 벽 안쪽 면 ±0.425, 두께 0.05."""
    yc = -d0
    walls = [box(-3, 6, yc+.425, yc+.475)]
    if wall_gap_right is None:
        walls.append(box(-3, 6, yc-right_wall_offset-.05, yc-right_wall_offset))
    else:
        g0, g1 = wall_gap_right
        walls += [box(-3, g0, yc-right_wall_offset-.05, yc-right_wall_offset),
                  box(g1, 6, yc-right_wall_offset-.05, yc-right_wall_offset)]
    walls += [box(6, 6.05, yc-.5, yc+.5), box(-3.05, -3, yc-.5, yc+.5)]   # 실내 코스처럼 끝을 막는다
    scene = cast(walls+[np.asarray(p) for p in extra], dropout=dropout)
    path = np.column_stack((np.linspace(-.25, 1.6, 36), np.full(36, yc)))
    return scene, path


def car(x_center, l_center, d0, length=.20, width=.15):
    """도로 중심 기준 횡위치 l_center의 상대차(base_link)."""
    y = l_center-d0
    return box(x_center-length/2, x_center+length/2, y-width/2, y+width/2)


# ------------------------------------------------------------------ 예산
def test_budget_values_and_policy_space_gain():
    b = LateralBudget()
    edge, detail = b.edge_margin()
    assert edge == pytest.approx(.03+.01+math.sqrt(.02**2+.01**2+.002**2))
    moving, _ = b.lateral_clearance(True)
    static, _ = b.lateral_clearance(False)
    assert moving == pytest.approx(.03+.15*.22+math.sqrt(.02**2+.01**2+.002**2))
    assert static < moving < .15          # 과거 15 cm 비교선과 별개
    limits = GRASS.policy_limit_m()
    assert limits['two_wheel_grass_m']-limits['road_only_m'] == pytest.approx(.0675+.075-.006)
    assert limits['road_only_m'] == pytest.approx(.15-edge)
    curved = GRASS.policy_limit_m(curved=True)
    assert curved['edge_margin_m'] > limits['edge_margin_m']
    with pytest.raises(ValueError):
        LateralBudget(range_bias_m=-.01)
    with pytest.raises(ValueError):
        LateralIntentConfig(surface_policy='all_grass')


def test_defaults_are_off_and_road_only():
    cfg = LateralIntentConfig()
    assert cfg.enabled is False and cfg.surface_policy == 'road_only'


def test_steering_protrusion_matches_wheel_geometry():
    assert steering_protrusion_m(0., GRASS) == 0.
    assert steering_protrusion_m(.45, GRASS) == pytest.approx(.0675+.0325*math.sin(.45)+.006*math.cos(.45)-.075)


# ------------------------------------------------------------------ 경로·노면
def test_path_frame_projection_on_curve():
    t = np.linspace(-.3, 1.2, 40)
    path = np.column_stack((np.sin(t), 1-np.cos(t)))     # 반경 1 왼쪽 곡선
    frame = PathFrame(path)
    s, l = frame.project(np.array([[0., -.1], [0., .1]]))
    assert l == pytest.approx([-.1, .1], abs=2e-3)
    pose = frame.pose(np.array([s[0]]), np.array([.2]))
    assert np.hypot(pose[0, 0], pose[0, 1]-1) == pytest.approx(.8, abs=3e-3)


def test_offset_candidate_folds_on_tight_inner_side_only():
    a = np.linspace(-.5, 4.2, 60)
    path = np.column_stack((.3*np.sin(a), .3*(1-np.cos(a))))   # 반경 0.3 m 왼쪽 급커브, 앞 약 1.26 m
    frame = PathFrame(path)
    state = ego_state(frame, GRASS)
    quick = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', blend_per_offset=2.0)
    inner = offset_candidate(frame, state, .25, quick)          # 안쪽 0.25 m: 1-κd = 0.17
    outer = offset_candidate(frame, state, -.2, quick)
    assert 'curvature_fold' in inner['problems']
    assert 'curvature_fold' not in outer['problems']


def test_surface_policies_road_only_versus_two_wheel_grass():
    path = np.column_stack((np.linspace(-.25, 1.6, 36), np.zeros(36)))
    frame = PathFrame(path)
    state = ego_state(frame, GRASS)
    for d, road_ok, grass_ok in ((-.05, True, True), (-.15, False, True), (-.20, False, True), (-.30, False, False)):
        cand = offset_candidate(frame, state, d, GRASS)
        assert surface_check(frame, cand, ROAD, current_pose=(0, 0, 0))['ok'] is road_ok, d
        result = surface_check(frame, cand, GRASS, current_pose=(0, 0, 0))
        assert result['ok'] is grass_ok, d
        if grass_ok and d < -.1:
            assert result['grass_side'] == 'right'


def test_road_only_allows_holding_existing_deficit_but_not_worsening():
    path = np.column_stack((np.linspace(-.25, 1.6, 36), np.full(36, .12)))   # 자차가 중심에서 -0.12
    frame = PathFrame(path)
    state = ego_state(frame, ROAD)
    assert state['d0'] == pytest.approx(-.12)
    hold = offset_candidate(frame, state, state['d0'], ROAD)
    worse = offset_candidate(frame, state, -.14, ROAD)
    assert surface_check(frame, hold, ROAD, current_pose=(0, 0, 0))['ok']
    assert not surface_check(frame, worse, ROAD, current_pose=(0, 0, 0))['ok']


def test_corridor_reference_detects_opening_and_nonstandard_wall():
    scene, path = straight(0.)
    frame = PathFrame(path)
    grid = ObservationGrid(scene, GRASS)
    ref = corridor_reference(frame, grid.returns, -.3, 1.2, GRASS)
    assert ref['left']['edge_verified'] and ref['right']['edge_verified']
    scene, path = straight(0., wall_gap_right=(.2, .5))
    ref = corridor_reference(PathFrame(path), ObservationGrid(scene, GRASS).returns, -.3, 1.2, GRASS)
    assert not ref['right']['edge_verified'] and ref['right']['max_gap_m'] >= .2
    scene, path = straight(0., right_wall_offset=.52)
    ref = corridor_reference(PathFrame(path), ObservationGrid(scene, GRASS).returns, -.3, 1.2, GRASS)
    assert not ref['right']['edge_verified']


# ------------------------------------------------------------------ 관측 격자·숨은 차량
def test_grid_keeps_rear_mask_and_unknown_unobserved():
    scene, path = straight(0., dropout=lambda deg: (deg >= 90) & (deg <= 150))
    grid = ObservationGrid(scene, GRASS)
    i, j, _ = grid.index(np.array([[-.6, 0.], [0., -.15], [0., .15], [.5, 0.]]))
    assert grid.state[i[0], j[0]] == UNOBS        # 후방 마스크
    assert grid.state[i[1], j[1]] == FREE         # 오른쪽 옆은 관측
    assert grid.state[i[2], j[2]] == UNOBS        # 왼쪽 옆은 unknown 광선
    assert grid.state[i[3], j[3]] == FREE
    k, m, _ = grid.index(np.array([[0., 0.]]))
    assert grid.state[k[0], m[0]] == EGO


def test_hidden_seeds_behind_but_not_beside_observed_front():
    scene, path = straight(0.)
    grid = ObservationGrid(scene, GRASS)
    frame = PathFrame(path)
    _, cell_l = cell_coordinates(grid, frame)
    mask = corridor_mask(cell_l, corridor_reference(frame, grid.returns, -.6, 1.2, GRASS), GRASS)
    seeds = grid.hidden_seeds(mask)
    ex, ey = grid.centers()
    xy = np.column_stack((ex[seeds], ey[seeds]))
    assert len(xy) and np.max(xy[:, 0]) < -.15        # 자차 뒤 마스크 안
    assert np.all(np.abs(xy[:, 1]) < .425)            # 확인된 벽 밖 제외


def test_hidden_tailgater_blocked_by_ego_body():
    cells = np.array([[-.05, .085]])
    tailgater = np.array([[-.25, 0.]])                # 자차 바로 뒤 같은 차로: 먼저 옆으로 0.125 m 빠져야 한다
    beside = np.array([[-.25, .17]])                  # 이미 왼쪽으로 빠진 차
    reach_t, t_t = hidden_reach(tailgater, cells, np.array([.5]), GRASS, .075)
    reach_b, t_b = hidden_reach(beside, cells, np.array([.5]), GRASS, .075)
    assert t_t[0] == pytest.approx(.125/.15+((.2-.075)-.075)/.5)
    assert t_b[0] == pytest.approx(max((.2-.075)/.5, (.085-.05)/.15))
    assert not reach_t[0] and reach_b[0]


# ------------------------------------------------------------------ 상태기계 반례
def run(scene, path, cfg=GRASS, speed=.35, limiter=None, previous=0.):
    limiter = limiter or LateralIntentLimiter(cfg)
    return limiter.step(scene, path, speed, previous_steering=previous), limiter


def test_empty_road_recenter_is_blocked_by_rear_blind_cone_unless_policy_accepts_it():
    """후방 60° 사각에 숨을 수 있는 차량(상대 0.5 m/s)은 중앙 복귀보다 먼저 옆에 닿을 수 있다."""
    scene, path = straight(-.09)
    hard, _ = run(scene, path, speed=0.)
    assert hard['mode'] == 'HOLD_OFFSET' and 'recenter_hidden_reach' in hard['reasons']
    assert hard['checks']['center']['verified_except_hidden']
    assert not hard['threats']['left']['present'] and not hard['threats']['right']['present']
    soft_cfg = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', recenter_hidden_policy='soft_after_quiet')
    soft, limiter = run(scene, path, soft_cfg, speed=.35)
    assert soft['mode'] == 'CENTER' and 'recenter_hidden_residual_risk_after_quiet' in soft['reasons']
    # 측방 위협을 본 직후에는 같은 정책이라도 중앙 복귀를 미룬다.
    side_scene, _ = straight(-.09, extra=[car(-.40, .13, -.09)])
    limiter.step(side_scene, path, .35)
    again = limiter.step(scene, path, .35)
    assert again['mode'] == 'HOLD_OFFSET'
    slow_cfg = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', hidden_relative_speed_mps=.1)
    slow, _ = run(scene, path, slow_cfg, speed=.35)
    fast, _ = run(scene, path, speed=.35)
    assert slow['checks']['center']['hidden_reach_cells'] < fast['checks']['center']['hidden_reach_cells']


def test_left_side_rear_car_yields_right_never_recenters():
    d0 = -.09
    scene, path = straight(d0, extra=[car(-.40, .13, d0)])
    out, _ = run(scene, path)
    assert out['threats']['left']['present']
    assert out['mode'] == 'YIELD' and out['checks']['yield']['d_target_m'] < d0
    target = out['checks']['yield']['d_target_m']
    assert .055-(target+.075) >= out['lateral_clearance_required_m']-1e-9
    assert out['selected'] != 'center'


def test_mirrored_right_car_yields_left():
    d0 = .09
    scene, path = straight(d0, extra=[car(-.40, -.13, d0)])
    out, _ = run(scene, path)
    assert out['threats']['right']['present'] and out['mode'] == 'YIELD'
    assert out['checks']['yield']['d_target_m'] > d0


def test_road_only_policy_cannot_open_enough_space():
    d0 = -.09
    scene, path = straight(d0, extra=[car(-.40, .13, d0)])
    out, _ = run(scene, path, ROAD)
    assert out['mode'] in ('HOLD_STOP', 'HOLD_OFFSET') and out['selected'] != 'center'
    if 'yield' in out['checks']:
        assert not out['checks']['yield']['verified']


def test_side_unknown_blocks_recenter_instead_of_trusting_absence():
    d0 = -.09
    scene, path = straight(d0, dropout=lambda deg: (deg >= 60) & (deg <= 150))
    out, _ = run(scene, path, speed=1.0)
    assert out['mode'] == 'HOLD_OFFSET'
    assert 'lateral_intent_unverified' in out['checks']['center']['reasons']


def test_corner_fixed_wall_is_not_a_side_threat():
    t = np.linspace(-.25, 1.6, 36)
    radius = 1.0
    path = np.column_stack((radius*np.sin(t/radius), radius*(1-np.cos(t/radius))))
    rings = []
    for r_in, r_out in ((radius-.475, radius-.425), (radius+.425, radius+.475)):
        a = np.linspace(-1.2, 2.5, 120)
        outer = np.column_stack((r_out*np.sin(a), radius-r_out*np.cos(a)))
        inner = np.column_stack((r_in*np.sin(a[::-1]), radius-r_in*np.cos(a[::-1])))
        rings.append(np.vstack((outer, inner)))
    scene = cast(rings)
    out, _ = run(scene, path)
    assert not out['threats']['left']['present'] and not out['threats']['right']['present']
    assert out['mode'] == 'CENTER'


def test_side_to_front_entry_never_steers_toward_it():
    d0 = 0.
    scene, path = straight(d0, extra=[car(.25, .17, d0)])     # 앞왼쪽에서 들어오는 차
    out, _ = run(scene, path)
    assert out['threats']['left']['present']
    sel = out['selected']
    assert sel is None or out['checks'][sel]['d_target_m'] <= d0+1e-9


def test_invalid_transition_does_not_fall_back_to_center():
    d0 = -.09
    scene, path = straight(d0, extra=[car(-.40, .13, d0)])
    limiter = LateralIntentLimiter(GRASS)
    first, _ = run(scene, path, limiter=limiter)
    assert first['mode'] == 'YIELD'
    short = path[:9]                       # 앞 관측 경로가 0.65 m 미만으로 줄어듦
    out, _ = run(scene, short, limiter=limiter, previous=.12)
    assert out['mode'] == 'HOLD_STOP' and out['selected'] is None
    assert out['braking_steering']['steering_rad'] <= 0.     # 위협(왼쪽) 쪽 조향을 고르지 않음
    missing = limiter.step(scene, None, .35)
    assert missing['mode'] == 'NO_BASE_PATH' and limiter.intent_offset is not None


def test_opening_on_grass_side_blocks_grass_yield():
    d0 = -.09
    scene, path = straight(d0, extra=[car(-.40, .13, d0)], wall_gap_right=(-.4, .9))
    out, _ = run(scene, path)
    assert out['mode'] != 'YIELD'
    reasons = out['checks'].get('yield', {}).get('reasons', [])
    assert 'grass_edge_reference_unverified' in reasons or 'lateral_clearance_short' in reasons


def test_both_sides_occupied_is_squeeze_not_yield():
    d0 = 0.
    scene, path = straight(d0, extra=[car(-.3, .25, d0), car(-.3, -.25, d0)])
    out, _ = run(scene, path)
    assert out['threats']['left']['present'] and out['threats']['right']['present']
    assert out['mode'] in ('SQUEEZE_HOLD_OFFSET', 'HOLD_STOP') and 'yield' not in out['checks']


def test_faster_hidden_bound_is_never_more_permissive():
    d0 = -.12
    scene, path = straight(d0)
    verdicts = []
    for v in (.5, 1.0, 2.0, 4.0):
        cfg = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass', hidden_relative_speed_mps=v)
        out, _ = run(scene, path, cfg, speed=1.0)
        verdicts.append(out['checks']['center']['verified'])
    assert verdicts == sorted(verdicts, reverse=True)


def test_stopped_center_car_bypass_needs_grass_policy():
    d0 = 0.
    scene, path = straight(d0, extra=[car(.9, 0., d0)])
    for cfg, expected in ((ROAD, False), (GRASS, True)):
        frame = PathFrame(path)
        state = ego_state(frame, cfg)
        cand = bypass_candidate(frame, state, ObservationGrid(scene, cfg).returns, 'right', cfg, stationary=True)
        assert cand is not None
        assert surface_check(frame, cand, cfg, current_pose=(0, 0, 0))['ok'] is expected


def test_coarse_cell_coordinates_match_exact_projection_on_curve():
    t = np.linspace(-.25, 1.6, 36)
    radius = .5
    path = np.column_stack((radius*np.sin(t/radius), radius*(1-np.cos(t/radius))))
    scene, _ = straight(0.)
    grid = ObservationGrid(scene, GRASS)
    frame = PathFrame(path)
    s4, l4 = cell_coordinates(grid, frame)
    s1, l1 = cell_coordinates(grid, frame, stride=1)
    # 반경 0.5 m 곡선의 곡률 중심(투영 자체가 불연속인 곳)은 벽(0.425 m) 바깥이다. 통로 안만 비교한다.
    # 경로가 크게 감기면 양 끝 연장선 투영이 겹쳐 불연속이 생긴다(경로 범위 밖). 범위 안만 비교한다.
    # 통로·위협 판정이 쓰는 범위는 벽 안쪽(|l| <= 0.425-0.03)이다.
    near = (np.abs(l1) <= .42) & (s1 >= 0) & (s1 <= frame.end_s)
    # 곡률 중심 쪽에서는 s의 기울기가 R/(R-l)배로 커져 보간 오차가 커진다(반경 0.5 m·l 0.42 m에서 약 4 cm).
    assert np.max(np.abs(l4-l1)[near]) < .003 and np.max(np.abs(s4-s1)[near]) < .05
