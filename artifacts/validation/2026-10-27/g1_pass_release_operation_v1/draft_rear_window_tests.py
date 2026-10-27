"""(초안) rear/unknown 해제의 실제 기하 반례. run1 사후 검사 뒤 tests/test_pass_release_trial.py로 옮긴다."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


def output(speed=1., angle=0., reason='local_follow'):
    return speed, angle, {'reason': reason, 'path_points': [[0., 0.], [1., 0.]]}


def box_scene(first, boxes=()):
    """벽 y=±0.425와 자차 좌표 축정렬 상자(x0,x1,y0,y1)의 광선 투사. 후방 ±30°는 기존 시험처럼 무반환."""
    raw = scan(first)
    angles = raw.angle_min+np.arange(500)*raw.angle_increment
    values = []
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        value = min(.425/max(abs(s), 1e-12), 8.)
        for x0, x1, y0, y1 in boxes:
            lo, hi = 0., value
            for o, d, a0, a1 in ((.06, c, x0, x1), (0., s, y0, y1)):
                if abs(d) < 1e-12:
                    if not a0 <= o <= a1:
                        lo, hi = 1., 0.
                    continue
                t0, t1 = sorted(((a0-o)/d, (a1-o)/d))
                lo, hi = max(lo, t0), min(hi, t1)
            if lo <= hi and lo > 0:
                value = min(value, lo)
        if abs(a) > math.radians(150)+1e-12:
            value = math.inf
        values.append(value)
    raw.ranges = values
    return motion_scene(raw, history(), first+.1, timing_verified=True)


STRAIGHT = np.c_[np.linspace(-1., 2., 61), np.zeros(61)].tolist()


def baseline(speed=1., angle=0.):
    return speed, angle, {'reason': 'local_follow', 'path_points': STRAIGHT}


BESIDE = (-.05, .15, .15, .30)          # 옆, 벽과 떨어진 compact 차량 크기 상자
MERGED_BEHIND = (-.40, -.20, .15, .425)  # 옆뒤, 벽에 붙어 compact가 꺼지는 물체
PASSING = (.00, .20, .12, .27)           # 다른 물체가 옆을 지나감
PASSED = (.22, .42, .12, .27)           # 최후단이 앞면+0.10 m를 넘은 통과(단독이면 benign)
AHEAD = (.60, .80, .12, .27)             # 측방 창 앞으로 빠짐
HIDDEN = (-.60, -.40, -.05, .10)         # 후방 마스크 원뿔 안에 완전히 숨음


def view_side(scene):
    return ps.threat_view(scene, STRAIGHT, ps.lateral_config(config()))['left']


def test_wall_merged_object_behind_is_not_present_and_blocks_only_the_whole_window():
    cfg, sel = config(), ps.SelectorConfig()
    lat = ps.lateral_config(cfg)
    scene = box_scene(1., [MERGED_BEHIND])
    assert not view_side(scene)['present']
    frame = ps.mx.ExecutionFrame(np.asarray(STRAIGHT))
    short = ps.band_check(scene, frame, 1., lat, sel)
    whole = ps.band_check(scene, frame, 1., lat, sel, (-lat.band_behind_m, lat.band_ahead_m))
    assert short['free'] and not whole['free'] and whole['unobserved'] > 0
    hidden = box_scene(1., [HIDDEN])
    clear = ps.band_check(hidden, frame, 1., lat, sel, (-lat.band_behind_m, lat.band_ahead_m))
    # 원뿔 안 물체는 배제하지 못한다. 마스크 표본으로 따로 셀 뿐 자유 관측으로 바꾸지 않는다.
    assert clear['free'] and clear['masked'] > 0 and clear['observed_free'] < clear['samples']


def test_real_geometry_rear_record_survives_second_object_and_needs_observed_window():
    ctl = ps.PassReleaseSelector(config())
    t = [1.]

    def step(boxes, base=None, previous=0.):
        t[0] += .1
        return ctl.select(box_scene(t[0], boxes), base or baseline(), (.2, 0., {'reason': 'parallel_hold_follow'}),
                          .3, previous, .02)
    assert view_side(box_scene(1., [BESIDE]))['present']
    step([BESIDE])
    assert ctl.mode == 'GUARD'
    for _ in range(3):
        out = step([MERGED_BEHIND])
    assert ctl.sides['left']['vanish'] == 'rear' and ctl.mode == 'GUARD' and out[0] == .2
    assert out[2]['selector_evidence']['left']['band']['free']
    assert not out[2]['selector_evidence']['left']['side_window']['free']
    # 첫 물체가 옆뒤에 남은 채 다른 물체가 옆을 지나 앞으로 빠져도 rear 기록·GUARD 유지.
    step([MERGED_BEHIND, PASSING])
    assert ctl.sides['left']['present']
    step([MERGED_BEHIND, PASSED])
    assert ctl.sides['left']['present'] and ctl.sides['left']['last_present']['rearmost_ds_m'] >= .2
    for _ in range(3):
        out = step([MERGED_BEHIND, AHEAD])
    assert ctl.sides['left']['vanish'] == 'rear' and ctl.mode == 'GUARD' and out[0] == .2
    # 관측 가능한 측방 창이 비고 첫 물체가 마스크 원뿔에만 있으면 두 fresh 뒤 해제(잔여 위험 명시).
    step([HIDDEN, AHEAD])
    assert ctl.mode == 'GUARD'
    out = step([HIDDEN, AHEAD])
    assert ctl.mode == 'RELEASED' and ctl.sides['left']['vanish'] == 'rear' and out[0] == 1.
    assert out[2]['selector_hidden_rear_risk_unresolved'] and not out[2]['selector_unknown_filled']


@pytest.mark.parametrize('region,reason', [
    ({'unobserved': 1, 'masked_left': 0, 'masked_right': 0}, 'large_motion_region_unobserved'),
    ({'unobserved': 0, 'masked_left': 3, 'masked_right': 0, 'masked_tail_extent_m': .011, 'masked_front_samples': 0},
     'large_motion_masked_toward_left'),
    ({'unobserved': 0, 'masked_left': 3, 'masked_right': 0, 'masked_tail_extent_m': .002, 'masked_front_samples': 1},
     'large_motion_masked_toward_left')])
def test_rear_release_still_rejects_large_motion_without_observed_region(monkeypatch, region, reason):
    state = {'view': {'frame': None, 'state': None,
                      'left': {'present': True, 'points': 4, 'rearmost_ds_m': -.2, 'frontmost_ds_m': 0.},
                      'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}}
    free = {'free': True, 'samples': 40, 'observed_free': 36, 'masked': 4, 'unobserved': 0}
    monkeypatch.setattr(ps, 'threat_view', lambda *a: state['view'])
    monkeypatch.setattr(ps, 'band_check', lambda *a: free)
    ctl = ps.PassReleaseSelector(config())

    def call(t, base=output(), previous=0.):
        return ctl.select(SimpleNamespace(source_first_s=t-.1, t_ref_s=t, source_epoch=0), base,
                          output(.2), .3, previous, .02)
    call(1.)
    state['view'] = {**state['view'], 'left': {'present': False, 'points': 0, 'rearmost_ds_m': None,
                                               'frontmost_ds_m': None}}
    call(1.1)
    call(1.2)
    assert ctl.mode == 'RELEASED' and ctl.sides['left']['vanish'] == 'rear'
    monkeypatch.setattr(ps, 'motion_region', lambda *a: region)
    out = call(1.3, base=output(angle=.25))
    assert out[0] == .2 and out[2]['selector_choice'] == 'parallel' and out[2]['selector_reason'] == reason


@pytest.mark.parametrize('fault', ['rewind', 'epoch', 'stale', 'invalid_speed'])
def test_rear_release_keeps_time_and_input_fault_rejections(monkeypatch, fault):
    views = {'present': {'frame': None, 'state': None,
                         'left': {'present': True, 'points': 4, 'rearmost_ds_m': -.2, 'frontmost_ds_m': 0.},
                         'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}},
             'empty': {'frame': None, 'state': None,
                       'left': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None},
                       'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}}
    state = {'view': views['present']}
    monkeypatch.setattr(ps, 'threat_view', lambda *a: state['view'])
    monkeypatch.setattr(ps, 'band_check', lambda *a: {'free': True, 'samples': 40, 'observed_free': 40,
                                                      'masked': 0, 'unobserved': 0})
    ctl = ps.PassReleaseSelector(config())

    def call(t, epoch=0, age=.1, speed=.3):
        return ctl.select(SimpleNamespace(source_first_s=t-age, t_ref_s=t, source_epoch=epoch), output(), output(.2),
                          speed, 0., .02)
    call(1.)
    state['view'] = views['empty']
    call(1.1)
    call(1.2)
    assert ctl.mode == 'RELEASED'
    kwargs = {'rewind': {'t': 1.0}, 'epoch': {'t': 1.3, 'epoch': 1}, 'stale': {'t': 1.3, 'age': .3},
              'invalid_speed': {'t': 1.3, 'speed': math.nan}}[fault]
    out = call(**kwargs)
    assert out[0] == 0. and out[2]['selector_choice'] == 'hold_fault'
    assert ctl.mode == 'GUARD' and all(s['vanish'] == 'unknown' for s in ctl.sides.values())
