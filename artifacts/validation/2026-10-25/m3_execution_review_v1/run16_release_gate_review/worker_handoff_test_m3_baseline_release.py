"""측방 시험의 기본 G1 복귀(선택) 관문. 반환 부재·점 개수 미달·fresh 2회만으로 해제하지 않는다.

해제는 마지막 위협의 가능 영역을 현재 스캔의 양의 free ray로 확인하고, 위협이 있을 수 있는 칸이 복귀 회랑·정지
거리 밖일 때만 일어난다. 해제 뒤 출력은 기본 G1이며 그때부터 측방 시험의 관측 계약(후방 마스크·측방 예산)은
적용되지 않는다. 10/25 run14의 '마지막 반환 ds ≥ 0.10 m + 2스캔' 판본은 탈락·가림·부분 관측을 부재로 치환해
반려됐고 그 소스·시험은 run14 스냅샷에 보존한다.
"""
from dataclasses import replace

import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_lateral_intent import GRASS, car, box
from test_local_follow import history
from test_m3_execution_trial import padding, ctl
from test_m3_observed_follow import timed_scene

D0 = -.09   # 공식 3번 슬롯과 같은 도로 횡위치
RELEASE = mx.ExecutionConfig(release_to_baseline=True)
SENSOR_X = .06


def car_x(t):
    return .30+.45*(t-1.0)       # 정지한 자차 기준 0.45 m/s로 앞서 나가는 왼쪽 옆차


def span_dropout(poly, keep=0):
    import math
    import numpy as np
    angles = [math.degrees(math.atan2(y, x-SENSOR_X)) for x, y in poly]
    lo, hi = min(angles)-1.5, max(angles)+1.5

    def fn(deg):
        inside = (deg >= lo) & (deg <= hi)
        if keep:
            idx = np.flatnonzero(inside)
            inside = inside.copy()
            inside[idx[len(idx)//2-keep//2: len(idx)//2-keep//2+keep]] = False
        return inside
    return fn


def run(sequence, execution=RELEASE, v=0.):
    trial = mx.ExecutionLateralTrial(GRASS, execution)
    h = history(v=v)
    outs = []
    for t, extra, dropout in sequence:
        scene, path = timed_scene(t, D0, extra=extra, dropout=dropout)
        outs.append((t, trial.step(scene, [], path, .35, padding(scene), history=h)))
    return trial, outs


def moving(after=None, n=13):
    seq = []
    for k in range(n):
        t = round(1.0+.1*k, 3)
        c = car(car_x(t), .13, D0)
        seq.append((t, [c], None) if k == 0 or after is None else after(t, c))
    return seq


def first_release(outs):
    return next((t for t, o in outs if o.get('released_to_baseline')), None)


def test_release_option_is_explicit_bool_and_default_false():
    assert mx.ExecutionConfig().release_to_baseline is False
    with pytest.raises(ValueError):
        mx.ExecutionConfig(release_to_baseline=1)


def test_side_return_positions_match_precheck_counts():
    scene, path = timed_scene(1., D0, extra=[car(.30, .13, D0)])
    frame = mx.ExecutionFrame(path)
    state = mx.ego_state(frame, GRASS)
    grid = mx.ReturnGrid(scene, GRASS)
    left, right = mx.side_return_ds(frame, state, grid, GRASS)
    assert mx.side_points(frame, state, grid, GRASS) == (len(left), len(right))
    assert len(left) and not len(right) and min(left) > GRASS.length_m/2


def test_clear_pass_releases_only_after_region_seen_and_outside_return_corridor():
    trial, outs = run(moving())
    released = first_release(outs)
    assert released is not None and not trial.active
    for t, o in outs:
        if t < released and not o['threats']['left']['present']:
            assert o['mode'] != 'BASELINE'
            assert o['release_gate']['reason'] in ('debounce', 'threat_possible_in_return_corridor',
                                                  'threat_region_unobserved')
    # 해제 시점에 옆차 뒤면은 자차 앞면 + 기본 G1 상한 1 m/s 정지 거리보다 앞이다(합성 정답).
    gate = dict(outs)[released]['release_gate']
    stop = gate['sides']['left']['stop_distance_m']
    assert car_x(released)-.10 > GRASS.length_m/2+stop
    assert gate['corridor_cells'] == 0 and gate['blocked_cells'] == 0


@pytest.mark.parametrize('name, after', [
    ('dropout', lambda t, c: (t, [c], span_dropout(c))),                                   # 무응답
    ('board_occlusion', lambda t, c: (t, [c, box(-.6, 1.6, .100, .110)], None)),          # 긴 비compact 가림
    ('two_points', lambda t, c: (t, [c], span_dropout(c, keep=2))),                       # 3점 미만
])
def test_disappearance_without_free_rays_never_releases(name, after):
    """run14 판본은 세 경우 모두 1.2 s에 해제했다(옆차 뒤면이 아직 측방 창 안 0.29 m)."""
    trial, outs = run(moving(after))
    assert first_release(outs) is None and trial.active
    reasons = {o['release_gate']['reason'] for t, o in outs if t >= 1.2}
    assert reasons <= {'threat_region_unobserved', 'record_stale_or_inconsistent'}
    assert any(o['release_gate']['blocked_cells'] > 0 for t, o in outs if 1.1 <= t <= 1.5)


def test_threat_lost_beside_in_rear_mask_never_releases_and_record_expires():
    c = car(0., .13, D0)
    seq = [(1.0, [c], None)]+[(round(1.0+.1*k, 3), [c], span_dropout(c)) for k in range(1, 13)]
    trial, outs = run(seq)
    assert first_release(outs) is None and trial.active
    last = dict(outs)
    assert last[2.0]['release_gate']['reason'] == 'threat_region_unobserved'
    assert last[2.1]['release_gate']['reason'] == 'record_stale_or_inconsistent'   # 1.0 s 초과 기록


def test_repeated_scan_does_not_evaluate_or_release_and_regression_resets():
    trial, outs = run(moving()[:10])                 # 1.9 s까지: 아직 회랑 안
    scene, path = timed_scene(1.9, D0, extra=[car(car_x(1.9), .13, D0)])
    again = trial.step(replace(scene, t_ref_s=1.92), [], path, .35, padding(scene), history=history())
    assert not again['release_gate']['sides'] and again['mode'] != 'BASELINE'
    old, path = timed_scene(1.5, D0, extra=[car(car_x(1.5), .13, D0)])
    back = trial.step(old, [], path, .35, padding(old), history=history())   # 원본 시각 역행
    assert not trial.records and back['mode'] == 'BASELINE' and not trial.active


def test_other_side_threat_blocks_release_and_its_rear_loss_keeps_trial():
    right = car(-.15, -.26, D0)      # 오른쪽 뒤 옆(마스크 밖, 오른쪽 벽 반환과 떨어진 compact 차)
    def after(t, c):
        return (t, [c, right], None) if t >= 1.6 else (t, [c], None)
    trial, outs = run(moving(after))
    assert first_release(outs) is None and trial.active
    assert any(o['release_gate']['reason'] == 'threat_present' for t, o in outs if t >= 1.6)
    seq = moving(after)[:8]+[(round(1.0+.1*k, 3), [car(car_x(1.0+.1*k), .13, D0), right],
                              span_dropout(right)) for k in range(8, 13)]
    trial, outs = run(seq)
    assert first_release(outs) is None and 'right' in trial.records


def test_epoch_change_clears_records():
    trial, _ = run(moving()[:5])
    assert trial.records
    scene, path = timed_scene(1.5, D0, extra=[car(car_x(1.5), .13, D0)])
    epoch = replace(scene, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in scene.rays))
    trial.step(epoch, [], path, .35, padding(scene), history=history())
    assert set(trial.records) <= {'left'} and all(r.scene.source_epoch == 1 for r in trial.records.values())


def test_without_history_or_option_never_releases():
    trial, outs = run(moving(), execution=mx.ExecutionConfig())
    assert first_release(outs) is None and all('release_gate' not in o for _, o in outs)
    trial = mx.ExecutionLateralTrial(GRASS, RELEASE)
    for t, extra, _ in moving():
        scene, path = timed_scene(t, D0, extra=extra)
        out = trial.step(scene, [], path, .35, padding(scene))       # 운동 이력 없음
        assert out['mode'] != 'BASELINE'


def test_released_controller_output_equals_baseline_g1():
    """해제 뒤 출력은 같은 입력의 기본 G1과 같다. 해제 전 활성 중에는 0.35 m/s 시험 상한을 유지한다."""
    follow = ctl(RELEASE)
    reference = LocalFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    h = history(v=0.)
    active, released = [], []
    # 기본 G1의 처음 세 호출은 경계 확인 대기라 측방 상태기가 돌지 않는다. 준비 뒤 옆차를 등장시킨다.
    seq = [(round(.4+.1*k, 3), [], None) for k in range(3)]+[(round(.7+.1*k, 3), [car(car_x(1.0+.1*k), .13, D0)], None)
                                                             for k in range(14)]
    for now, extra, _ in seq:
        scene, _ = timed_scene(now, D0, extra=extra)
        got = follow.command(scene, h, 0., 'left')
        expected = reference.command(scene, h, 0., 'left')
        lateral = got[2].get('m3_lateral', {})
        if lateral.get('mode') not in (None, 'BASELINE'):
            active.append(now)
            assert got[0] <= .35+1e-12
        if lateral.get('released_to_baseline') or (released and lateral.get('mode') == 'BASELINE'):
            released.append(now)
            assert got[:2] == expected[:2] and got[2]['reason'] == expected[2]['reason']
    assert active and released and min(released) > max(active), (active, released)
