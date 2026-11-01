"""후보 A 스크래치 회귀. 저장소 tests가 아니며 소스 해제 뒤 옮길 항목의 초안이다.

실행: ARENA_CANDIDATE_TREE=<snap_candA> python3 -m pytest -q -p no:cacheprovider test_candidate_a.py
"""
import copy
import math
from collections import Counter

import numpy as np
import pytest

import support as h
from support import lf

TOL = .08


def polyline(*segments, step=.02):
    """(길이, 각도) 구간을 이어 붙인 경로. 차체 원점 바로 옆에서 시작한다."""
    points, position = [np.array([-.04, 0.])], np.array([-.04, 0.])
    for length, angle in segments:
        for _ in range(int(round(length/step))):
            position = position+step*np.array([math.cos(angle), math.sin(angle)])
            points.append(position)
    return np.array(points)


# ── 호길이 비교: 범위 요구와 분기 ────────────────────────────────────────────

def test_dense_short_paths_do_not_prove_common_range():
    # 점이 아무리 많아도 선택 호길이가 0.25 m 미만이면 비교 범위를 증명하지 못한다.
    path = polyline((.19, 0.), step=.001)
    assert len(path) > 150 and not lf.path_agrees_arc(path, path, TOL)


@pytest.mark.parametrize('length,expected', [(.33, False), (.35, True)])
def test_quarter_metre_boundary(length, expected):
    # 경로는 x=-0.04에서 시작하고 선택은 호길이 0.05부터다. 끝 호길이는 길이-0.04다.
    path = polyline((length, 0.))
    assert bool(lf.path_agrees_arc(path, path, TOL)) is expected


def test_common_range_uses_shorter_path():
    assert not lf.path_agrees_arc(polyline((1.2, 0.)), polyline((.25, 0.)), TOL)


def test_parallel_walls_keep_same_tolerance():
    base = polyline((1.2, 1.2))
    normal = np.array([-math.sin(1.2), math.cos(1.2)])
    assert lf.path_agrees_arc(base, base+.06*normal, TOL)
    assert not lf.path_agrees_arc(base, base+.15*normal, TOL)


@pytest.mark.parametrize('turn', [.5, 1.2])
@pytest.mark.parametrize('shared', [.25, .5])
def test_early_fork_is_rejected_in_any_heading(shared, turn):
    # 좌우 가지가 호길이 0.5 m 안에서 갈라지면 차체 방향과 무관하게 다른 경로다.
    a = polyline((shared, turn), (1.2, turn+.8))
    b = polyline((shared, turn), (1.2, turn-.8))
    assert not lf.path_agrees_arc(a, b, TOL)


def test_reversed_path_order_is_rejected():
    path = polyline((1.2, 1.2))
    assert not lf.path_agrees_arc(path, path[::-1], TOL)


def test_known_limit_late_fork_is_not_distinguished():
    """알려진 한계: 공통 80%를 넘긴 뒤 갈라지면 구별하지 못한다.

    기존 x 창도 0.65 m 밖의 분기는 보지 못한다. 이 판단은 벽 정체를 인증하지 않으며
    선호 측 경로 재획득과 이후 점유·재개 판정만 허용한다.
    """
    a, b = polyline((.85, 0.), (1., .8)), polyline((.85, 0.), (1., -.8))
    assert lf.path_agrees(a, b, TOL) and lf.path_agrees_arc(a, b, TOL)
    sideways = [polyline((.85, 1.3), (1., 2.1)), polyline((.85, 1.3), (1., .5))]
    assert not lf.path_agrees(*sideways, TOL) and lf.path_agrees_arc(*sideways, TOL)


# ── 저장 정지 장면 ────────────────────────────────────────────────────────

def test_prefix_replay_matches_recorded_decisions():
    assert h.state().mismatch == 0


def test_option_off_keeps_recorded_stop():
    rows = h.play(False)
    assert all(r.v == 0 and r.s == 0 for r in rows)
    assert {r.reason for r in rows} == {'follow_boundary_unconfirmed'}


def test_option_on_restarts_only_after_existing_release():
    rows = h.play(True)
    first = next(i for i, r in enumerate(rows) if r.v > 0)
    assert rows[first].reason == 'local_follow' and rows[first].published > .3
    # 기존 재개 규칙: 경로 수락 뒤 서로 다른 새 스캔 2장 이상이 먼저 필요하다.
    assert sum(1 for r in rows[:first] if r.fresh) >= 2
    assert any(r.bilateral == 'low_speed_arc' for r in rows)
    assert all(r.reason == 'follow_steer_in_place' for r in rows if r.v == 0 and r.detail.get('swept_clearance_m') is not None
               and r.detail['swept_clearance_m'] <= .2)


def test_recorded_scene_needs_arc_not_x_window():
    rows = h.play(True)
    accepted = [r for r in rows if r.bilateral == 'low_speed_arc']
    assert accepted and all(r.detail['boundary_selection'] == 'unfiltered' for r in accepted)


@pytest.mark.parametrize('speed,allowed', [(-.5, False), (-.11, False), (-.10, True), (-.08, True), (0., True),
                                           (.08, True), (.10, True), (.11, False), (.5, False)])
def test_condition_is_measured_magnitude(speed, allowed):
    rows = h.play(True, speed=speed)
    assert any(r.bilateral == 'low_speed_arc' for r in rows) is allowed
    assert any(r.v > 0 for r in rows) is allowed


def test_legacy_hold_negative_feedback_still_holds():
    rows = h.play(True, speed=-.08, signed_motion_policy='legacy_hold')
    assert all(r.v == 0 for r in rows) and {r.reason for r in rows} == {'follow_reverse_motion'}


def test_low_measured_speed_is_not_treated_as_proof_of_standstill():
    """낮은 측정값이어도 양수 출력은 새 스캔 2장의 재개 판정을 건너뛰지 못한다."""
    follow = h.controller(True)
    fresh_before_positive, previous, sources = 0, 0., set()
    for call in h.state().calls:
        data = copy.deepcopy(call)
        data['previous_steering_rad'] = previous
        data['speed_mps'] = .05     # 헛도는 동안에도 평균이 이 값일 수 있다.
        v, s, detail = h.replay_call(follow, data, h.state().motion)
        previous = max(previous-.06, min(previous+.06, s))
        sources.add(detail.get('source_first_s'))
        if v > 0:
            break
    assert v > 0 and len(sources) >= 3


# ── 관측 장애물과 독립 보호 ───────────────────────────────────────────────

def block_left_arc(data, distance=.34):
    """저장 스캔의 왼쪽 앞 조향 호에 실제 반환을 넣는다. 무반환을 자유 공간으로 바꾸지 않는다."""
    scan = data['scan']
    for i in range(len(scan['ranges'])):
        angle = scan['angle_min']+i*scan['angle_increment']
        if 0 <= angle <= math.radians(70):
            value = scan['ranges'][i]
            scan['ranges'][i] = min(distance, value) if isinstance(value, float) else distance


def test_observed_obstacle_on_steered_arc_keeps_zero():
    rows = h.play(True, inject=block_left_arc)
    assert all(r.v == 0 for r in rows)


def guard_series(distance):
    policy, values = copy.deepcopy(h.state().policy), []
    for call in h.state().calls[:30]:
        data = copy.deepcopy(call)
        if distance is not None:
            block_left_arc(data, distance)
        values.append(h.guard(policy, data, .5, .371, .371))
    return values


def test_independent_guard_decides_forward_command():
    clear, near, close = guard_series(None), guard_series(.34), guard_series(.16)
    assert all(v[0] == .5 and v[1] == 'clear' and v[2] > .9 for v in clear)
    # 보호층은 자기 입력으로 다시 계산한다. 가까운 반환일수록 여유와 허용 속도가 줄어든다.
    assert max(v[2] for v in near) < .5
    assert max(v[2] for v in close) < .2 and max(v[0] for v in close) < .35


def test_candidate_does_not_touch_guard_output_path():
    # 후보는 /drive 입력만 바꾼다. 패치는 local_follow.py 한 파일이다.
    patch = (h.HERE.parent/'candidate_a.patch').read_text(encoding='utf-8')
    changed = [line for line in patch.splitlines() if line.startswith('+++ ')]
    assert changed == ['+++ b/src/arena_autonomy/arena_autonomy/local_follow.py']
    assert 'lidar_safety' not in patch


# ── 합성 정지 장면: 후보가 새 양수 출력을 만들지 않아야 하는 경우 ─────────

ROOM = [(-6., -6., 6., -6.), (6., -6., 6., 6.), (6., 6., -6., 6.), (-6., 6., -6., -6.)]
SCENES = {
    'early_fork_0p3': h.fork(.3)+ROOM,
    'early_fork_0p6': h.fork(.6)+ROOM,
    'side_gap': [(-1., -.425, 3., -.425), (-1., .425, .2, .425), (1.3, .425, 3., .425), (-1., -.425, -1., .425)]+ROOM,
    'observed_block': h.corridor(((.38, -.425, .38, .425),))+ROOM,
}


@pytest.mark.parametrize('name', sorted(SCENES))
def test_new_branch_adds_no_motion_in_ambiguous_scene(name):
    off, on = h.play_scene(SCENES[name], False), h.play_scene(SCENES[name], True)
    assert [r.v for r in on] == [r.v for r in off]
    assert not any(r.bilateral == 'low_speed_arc' for r in on)


def test_front_no_return_stays_unconfirmed():
    off = h.play_scene(h.corridor()+ROOM, False, blind=((-.3, .3),))
    on = h.play_scene(h.corridor()+ROOM, True, blind=((-.3, .3),))
    assert [r.v for r in on] == [r.v for r in off] and all(r.v == 0 for r in on)
