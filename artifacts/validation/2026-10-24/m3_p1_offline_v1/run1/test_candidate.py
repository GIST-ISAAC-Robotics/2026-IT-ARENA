"""A2/P1 시제품의 독립 합성 단위 회귀. 저장 기록·정답을 쓰지 않는다."""
import inspect
import math
from pathlib import Path
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(HERE), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import candidate as c  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402
from arena_autonomy.local_path import swept_limit  # noqa: E402

CFG = c.A2Config()
FOLLOW = FollowConfig(corridor_fallback_enabled=True, boundary_evidence_mode='corner_distance_candidate',
                      occupancy_timing_mode='per_ray_candidate', lateral_acceleration_mps2=3.0)


def straight(y=0., x0=-.25, x1=1.5, n=36, heading=0.):
    t = np.linspace(0., x1-x0, n)
    return np.column_stack((x0*math.cos(heading)+t*math.cos(heading), y+x0*math.sin(heading)+t*math.sin(heading)))


def arc(radius, left=True, length=1.6, n=60, start=-.2):
    s = np.linspace(start, start+length, n)
    a = s/radius
    sign = 1. if left else -1.
    return np.column_stack((radius*np.sin(a), sign*radius*(1-np.cos(a))))


def no_rays():
    return np.zeros(0), np.zeros(0, dtype='<U8')


def test_p1_straight_offset_is_exact():
    result = c.p1_path(straight(), -.09, .075)
    assert result['valid'] and result['reason'] == 'ok'
    assert np.allclose(result['path'][:, 1], -.09)
    assert result['swept_footprint_in_road'] == 'unverified'


def test_p1_arc_normal_offset_radius():
    path = arc(.6)
    centre = np.array([0., .6])
    # 정적 도로 끝 선별과 무관한 곡선 기하만 본다(offset_curve는 진단용, 유효 경로 아님).
    inside = c.p1_path(path, .15, .075)['offset_curve']
    outside = c.p1_path(path, -.15, .075)['offset_curve']
    # 끝점 미분 오차를 피해 내부 점만 본다.
    assert np.allclose(np.linalg.norm(inside[3:-3]-centre, axis=1), .45, atol=2e-3)
    assert np.allclose(np.linalg.norm(outside[3:-3]-centre, axis=1), .75, atol=2e-3)


def test_p1_fold_inside_small_radius_is_invalid():
    result = c.p1_path(arc(.12, length=.9), .15, .075)
    assert not result['valid']
    assert {'curvature_fold', 'direction_reversal', 'self_intersection'} & set(result['problems'])


def test_p1_static_edge_screen_is_not_footprint_proof():
    result = c.p1_path(straight(), -.14, .075)   # 0.14+0.075 > 0.225-0.02
    assert not result['valid'] and 'static_edge_screen' in result['problems']
    ok = c.p1_path(straight(), -.09, .075)
    assert ok['valid'] and ok['swept_footprint_in_road'] == 'unverified'


def test_side_state_sign_and_body_exclusion():
    path = straight(y=.09)                         # 경로가 자차 왼쪽 → 자차는 경로 오른쪽
    points = np.array([[-.30, 0.], [-.30, .20], [-.30, -.20], [.2, .40]])
    state = c.side_state(points, *no_rays(), path, CFG)
    assert state['ego_offset_m'] == pytest.approx(-.09, abs=1e-9)
    assert state['centering_toward'] == 'left'
    assert state['sides']['left']['points'] == 2       # (-.3,.2), (.2,.4) [|l|=.31<.345]
    assert state['sides']['right']['points'] == 1
    assert state['inlane_rear_points'] == 1            # 바로 뒤 점은 측방으로 세지 않는다
    assert state['window_coverage']['status'] == 'unverifiable'
    assert 'rear_mask_overlap' in state['window_coverage']['reasons']


def test_static_parallel_box_hold_threshold_uses_existing_swept():
    """정적 7 cm 평행 상자: 팽창 0.08 m면 HOLD, 0.06 m면 아님(추론의 단위 확인)."""
    y = np.linspace(-.10, .10, 21)
    box = np.column_stack((y, np.full_like(y, .075+.07)))
    assert swept_limit(box, 0., .135, horizon=2.5, margin=np.full(len(box), .08)) <= .20
    assert swept_limit(box, 0., .135, horizon=2.5, margin=np.full(len(box), .06)) > .20


def presence_state(left_points, unknown_left=0, offset=-.09, psi_ok=True):
    return {'status': 'path', 'centering_toward': 'left', 'ego_offset_m': offset, 'geometry_valid': psi_ok,
            'sides': {'left': {'points': left_points}, 'right': {'points': 0}},
            'side_rear_unknown': {'left': unknown_left, 'right': 0}}


def test_machine_unknown_blocks_release_and_release_is_unverified():
    m = c.A2Machine(CFG)
    assert m.step(presence_state(5), True, True) == ('P1', ['trigger'])
    assert m.step(presence_state(0, unknown_left=4), True, True) == ('P1', ['release_blocked_unknown'])
    assert m.step(presence_state(0), True, True) == ('P1', [])
    mode, events = m.step(presence_state(0), True, True)
    assert mode == 'P0' and events == ['release_unverified']


def test_machine_stale_scans_do_not_count_for_release():
    m = c.A2Machine(CFG)
    m.step(presence_state(5), True, True)
    for _ in range(5):
        assert m.step(presence_state(0), True, False)[0] == 'P1'


def test_invalid_transition_resumes_steering_toward_threat():
    """P1 활성 뒤 코너 진입(경로 대비 yaw 20°)으로 기하 무효 → P0 복귀 조향이 위협(왼쪽) 쪽."""
    m = c.A2Machine(CFG)
    base = straight(y=.09)
    s1 = c.side_state(np.array([[-.30, .22], [-.32, .22], [-.34, .22]]), *no_rays(), base, CFG)
    p1 = c.p1_path(base, s1['ego_offset_m'], s1['half_span_m'], CFG)
    assert m.step(s1, p1['valid'], True) == ('P1', ['trigger'])
    corner = straight(y=.09, heading=math.radians(20))       # 위협은 이제 마스크 안이라 보이지 않음
    s2 = c.side_state(np.zeros((0, 2)), *no_rays(), corner, CFG)
    assert not s2['geometry_valid'] and 'heading_exceeds_screen' in s2['geometry_reasons']
    mode, events = m.step(s2, True, True)
    assert mode == 'P0' and events == ['invalid_transition:geometry']
    p0 = c.existing_checks(corner, np.array([[3., 3.]]), np.array([.055]), .4, 0., .1, FOLLOW)
    assert p0['status'] == 'ok' and c.toward_threat(p0['steering_rad'], 'left')


def test_candidate_signatures_have_no_truth_inputs():
    banned = ('actor', 'truth', 'pose', 'schedule', 'ego_pose', 'world')
    for fn in (c.side_state, c.p1_path, c.existing_checks, c.A2Machine.step, c.window_coverage):
        params = inspect.signature(fn).parameters
        assert not any(b in name for name in params for b in banned)
