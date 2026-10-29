"""G2 첫 진입 비교의 센서/명령/범위 계약. 우회 주행 성공 테스트가 아니다."""
from dataclasses import replace
import math
import numpy as np
import pytest

from arena_autonomy.bypass_entry_trial import (EntryConfig, compare_entries, entry_path,
                                               preview_clearance, straight_reference, EntryIntent)
from arena_autonomy.local_follow import FollowConfig
from probe_bypass_entry import sensor_scene


def evaluate(**kwargs):
    scene, _ = sensor_scene(**kwargs)
    return compare_entries(scene, scene.t_ref_s, 0., 0., .02)


def test_empty_lane_uses_measured_two_walls_and_no_route_offset():
    out = evaluate()
    assert out['reference']['reason'] == 'current_two_wall_straight_reference'
    assert out['selected'] is not None and out['selected']['offset_m'] == 0.
    assert not out['control_authorized'] and not out['unknown_filled']
    assert not out['full_bypass_verified'] and not out['road_surface_verified']
    assert len(out['candidates']) == 19


@pytest.mark.parametrize('targets', [((.22, 0.),), ((.2, -.05), (.2, .05))])
def test_close_front_returns_cannot_be_ignored_for_lateral_entry(targets):
    out = evaluate(targets=targets)
    assert out['selected'] is None
    assert not any(row['eligible'] for row in out['candidates'])


@pytest.mark.parametrize('speed,previous,dt,now', [
    (-.01, 0., .02, .5), (1.01, 0., .02, .5), (0., .4, .02, .5),
    (0., 0., 0., .5), (0., 0., .21, .5), (0., 0., .02, .6),
    (math.nan, 0., .02, .5), (0., math.inf, .02, .5), (True, 0., .02, .5)])
def test_invalid_motion_is_not_a_candidate(speed, previous, dt, now):
    scene, _ = sensor_scene()
    out = compare_entries(scene, now, speed, previous, dt)
    assert out['selected'] is None and out['reason'] == 'invalid_input'


def test_stale_scan_is_not_refreshed_to_reference_time():
    assert evaluate(now=.71)['selected'] is None


def test_front_missing_is_preserved_even_with_no_occupied_target():
    out = evaluate(missing=tuple(range(230, 271)))
    assert out['selected'] is None
    assert all(not r['eligible'] for r in out['candidates'])


def test_wall_reference_cannot_be_extrapolated_to_finish_transition():
    path = entry_path((0., 0., -.1, .8), .16, FollowConfig(), EntryConfig())
    assert path[-1,0] == .8
    assert 0 < path[-1,1] < .16
    with pytest.raises(ValueError, match='prefix too short'):
        entry_path((0., 0., -.1, .20), .16, FollowConfig(), EntryConfig())


def test_path_starts_at_current_rear_and_stays_inside_observed_reference():
    config, entry = FollowConfig(), EntryConfig()
    path = entry_path((0., .02, -.12, 1.4), -.16, config, entry)
    assert path[0] == pytest.approx([-config.wheelbase_m/2, 0.])
    assert path[-1] == pytest.approx([1.4, -.14])
    assert np.all(np.diff(path[:, 0]) > 0)
    assert np.max(np.diff(path[:, 0])) <= entry.path_step_m+1e-12


def test_preview_keeps_every_current_occupied_point():
    config = FollowConfig()
    path = np.c_[np.linspace(-.0675, 1.2, 100), np.zeros(100)]
    far = np.array([[.5, .4], [.5, -.4]])
    before = preview_clearance(path, far, np.full(2, .08), config)
    near = np.r_[far, [[.35, 0.]]]
    after = preview_clearance(path, near, np.full(3, .08), config)
    assert after < before and after < .2


def test_low_target_is_explicitly_not_detected_by_horizontal_scan():
    empty_scene, _ = sensor_scene()
    low_scene, evidence = sensor_scene(targets=((.8, 0.),), target_height=.06)
    assert evidence['target_returns'] == 0
    assert low_scene == empty_scene
    assert not evidence['low_obstacle_detection_verified']


@pytest.mark.parametrize('kwargs', [dict(offsets_m=(0., .28)), dict(offsets_m=(.1,)),
    dict(offsets_m=(0., 0.)), dict(offsets_m=(0., math.nan)), dict(speed_cap_mps=.5),
    dict(transitions_m=(.2,)), dict(transitions_m=(1.2, .6)), dict(path_step_m=.1)])
def test_comparison_configuration_bounds(kwargs):
    with pytest.raises(ValueError): EntryConfig(**kwargs)


def test_command_rate_limit_and_age_padding_are_visible_in_candidates():
    out = evaluate(targets=((1.2, .15),))
    assert out['candidates']
    for row in out['candidates']:
        if 'steering_rad' in row:
            assert abs(row['steering_rad']) <= .06+1e-12
            assert row['maximum_occupancy_padding_m'] >= .055
            assert row['target_speed_mps'] <= .35
            assert not row['control_authorized']
            assert not row['path_extrapolated_beyond_current_wall']
            assert row['path_points'][-1][0] <= row['observed_reference_x_m'][1]


def test_input_scene_is_not_modified():
    scene, _ = sensor_scene(targets=((.8, .15),))
    before = repr(scene)
    compare_entries(scene, .5, 0., 0., .02)
    assert repr(scene) == before


def test_empty_preview_does_not_reward_longer_curved_path():
    config, entry = FollowConfig(), EntryConfig()
    points = np.array([[3., 3.]])
    straight = entry_path((0., 0., -.1, 1.4), 0., config, entry)
    curved = entry_path((0., 0., -.1, 1.4), .20, config, entry)
    assert preview_clearance(straight, points, np.array([.1]), config) == pytest.approx(
        preview_clearance(curved, points, np.array([.1]), config))


def test_each_eligible_candidate_respects_preview_steering_limit():
    out = evaluate(targets=((1.2, 0.),))
    assert all(row['preview_maximum_steering_rad'] <= .37
               for row in out['candidates'] if row['eligible'])


def rotated_scene(angle):
    scene, _ = sensor_scene()
    c, s = math.cos(angle), math.sin(angle)
    def rotate(p):
        return (c*p[0]+s*p[1], -s*p[0]+c*p[1])
    rays=tuple(replace(r, ray_origin_xy_m=rotate(r.ray_origin_xy_m),
        hit_xy_m=None if r.hit_xy_m is None else rotate(r.hit_xy_m),
        direction_xy=rotate(r.direction_xy)) for r in scene.rays)
    return replace(scene, rays=rays)


@pytest.mark.parametrize('angle', [-.3, -.15, .15, .3])
def test_straight_road_remains_straight_when_car_yaws(angle):
    reference, evidence = straight_reference(rotated_scene(angle), FollowConfig())
    assert reference is not None
    assert evidence['reason'] == 'current_two_wall_straight_reference'


def intent_comparison(left=.9, right=.8, straight=.4, *, left_ok=True, right_ok=True):
    rows=[dict(offset_m=offset, transition_m=1.2, eligible=eligible,
        preview_return_progress_m=distance, target_speed_mps=.2, steering_rad=offset)
        for offset,distance,eligible in ((0.,straight,True),(.16,left,left_ok),(-.16,right,right_ok))]
    chosen=max([r for r in rows if r['eligible']],key=lambda r:r['preview_return_progress_m'])
    return dict(candidates=rows, selected=dict(offset_m=chosen['offset_m'], transition_m=1.2,
        speed_mps=.2, steering_rad=chosen['steering_rad']), reason='entry_command_candidates_available',
        control_authorized=False, unknown_filled=False)


def test_intent_prevents_opposite_side_flip_but_never_reuses_permission():
    state=EntryIntent();scene,_=sensor_scene()
    a=state.select(scene,intent_comparison())
    assert a['selected']['offset_m']>0
    b=state.select(scene,intent_comparison(left=.8,right=1.2))
    assert b['selected']['offset_m']>0
    c=state.select(scene,intent_comparison(left_ok=False))
    assert c['selected'] is None and c['reason']=='entry_committed_side_currently_unavailable'
    assert not c['intent_remembered_permission'] and not c['control_authorized']


def test_intent_keeps_straight_for_less_than_two_preview_steps_gain():
    state=EntryIntent();scene,_=sensor_scene()
    out=state.select(scene,intent_comparison(left=.42,right=.4,straight=.4))
    assert out['selected']['offset_m']==0 and state.sign is None


@pytest.mark.parametrize('change', ['epoch','time','source'])
def test_intent_is_cleared_after_source_discontinuity(change):
    state=EntryIntent();scene,_=sensor_scene()
    state.select(scene,intent_comparison())
    if change=='epoch':different,_=sensor_scene(epoch=1)
    elif change=='time':different,_=sensor_scene(now=.49,first=.3)
    else:different,_=sensor_scene(now=.51,first=.3)
    out=state.select(different,intent_comparison())
    assert state.sign is None and out['selected'] is None
    assert out['reason']=='entry_intent_input_reset'


def test_short_inner_wall_agreement_does_not_cut_off_observed_outer_path(monkeypatch):
    import arena_autonomy.bypass_entry_trial as module
    def observed(points, side, **kwargs):
        end=1.3 if side=='left' else .6
        x=np.linspace(-.12,end,65)
        return np.c_[x,np.zeros_like(x)], {}
    monkeypatch.setattr(module,'local_path',observed)
    scene,_=sensor_scene()
    out=compare_entries(scene,.5,0.,0.,.02)
    assert out['reference']['reason']=='current_two_wall_straight_reference'
    left=[r for r in out['candidates'] if r['offset_m']>0 and 'path_points' in r]
    right=[r for r in out['candidates'] if r['offset_m']<0 and 'path_points' in r]
    assert left and right
    assert all(r['path_points'][-1][0]==1.3 for r in left)
    assert all(r['path_points'][-1][0]==.6 for r in right)
    assert any(not r['transition_reaches_offset'] for r in right)


def test_short_inner_wall_still_requires_two_wall_agreement(monkeypatch):
    import arena_autonomy.bypass_entry_trial as module
    def observed(points, side, **kwargs):
        x=np.linspace(-.12,1.3 if side=='left' else .6,65)
        return np.c_[x,np.full_like(x,.10 if side=='right' else 0.)], {}
    monkeypatch.setattr(module,'local_path',observed)
    assert evaluate()['reason']=='observed_wall_reference_disagrees'


def test_encoder_conversion_roundoff_does_not_make_nominal_cap_invalid():
    scene,_=sensor_scene()
    measured=(.35/.0325)*.0325
    assert compare_entries(scene,.5,measured,0.,.02)['reason']!='invalid_input'
    assert compare_entries(scene,.5,.350000001,0.,.02)['reason']!='invalid_input'
    assert compare_entries(scene,.5,1.000001,0.,.02)['reason']=='invalid_input'


def test_entry_speed_preserves_reaction_and_braking_distance():
    for clearance in (0., .04, .06, .12, .16, .20, .4):
        from arena_autonomy.local_path import braking_speed
        for age in (.1, .18, .29):
            target=min(.35,braking_speed(clearance,age))
            if target>0:
                assert target*(age+.1)+target**2/4+.06 <= clearance+1e-12
            else:
                assert clearance<=.06


def test_entry_does_not_remove_current_occupied_returns():
    out=evaluate(targets=((.18,0.),))
    assert out['selected'] is None


def test_duplicate_commands_share_work_only_within_the_current_call(monkeypatch):
    import arena_autonomy.bypass_entry_trial as module
    swept, region = module.swept_limit, module.motion_region
    angles, commands = [], []
    def count_swept(points, angle, *args, **kwargs):
        angles.append(angle)
        return swept(points, angle, *args, **kwargs)
    def count_region(scene, speed, target, steer, *args, **kwargs):
        commands.append((steer, target))
        return region(scene, speed, target, steer, *args, **kwargs)
    monkeypatch.setattr(module, 'swept_limit', count_swept)
    monkeypatch.setattr(module, 'motion_region', count_region)
    first=evaluate(targets=((1.2,0.),))
    assert len(angles)==len(set(angles))
    assert len(commands)==len(set(commands))
    assert len(angles) < 2*sum('steering_rad' in row for row in first['candidates'])
    before=len(angles)
    # 같은 입력으로 호출해도 호출 간 캐시는 없고 거리를 다시 검사한다.
    again=evaluate(targets=((1.2,0.),))
    assert len(angles)==2*before and again==first
    # 새 가까운 반환은 앞 호출의 허가를 재사용하지 않는다. 기준 생성에서
    # 먼저 거절되면 명령 원호 검사까지 도달하지 않을 수도 있다.
    close=evaluate(targets=((.18,0.),))
    assert close['selected'] is None


@pytest.mark.parametrize('measured', [.364649147291673, .8])
def test_speed_measurement_above_command_cap_is_not_clipped(monkeypatch, measured):
    import arena_autonomy.bypass_entry_trial as module
    seen=[]
    original=module.entry_motion_region
    def capture(scene,speed,target,*args,**kwargs):
        seen.append((speed,target))
        return original(scene,speed,target,*args,**kwargs)
    monkeypatch.setattr(module,'entry_motion_region',capture)
    scene,_=sensor_scene()
    out=compare_entries(scene,.5,measured,0.,.02)
    assert out['reason']!='invalid_input' and seen
    assert all(speed==measured and 0<=target<=.35 for speed,target in seen)
    for row in out['candidates']:
        if 'motion_region' in row:
            assert row['motion_region']['distance_m'] > 0
    near,_=sensor_scene(targets=((.18,0.),))
    assert compare_entries(near,.5,measured,0.,.02)['selected'] is None


def test_earlier_transition_tiebreak_preserves_observed_clearance_and_steering():
    from arena_autonomy.bypass_entry_trial import _compare_entries
    scene,_=sensor_scene(targets=((1.2,0.),))
    baseline=_compare_entries(scene,.5,0.,0.,.02,stationary_boundaries=True)
    early=_compare_entries(scene,.5,0.,0.,.02,stationary_boundaries=True,
        entry=EntryConfig(prefer_earlier_transition=True))
    assert baseline['selected']['offset_m']==early['selected']['offset_m']==.24
    assert baseline['selected']['transition_m']==1.2
    assert early['selected']['transition_m']==.9
    chosen=early['selected']
    row=next(row for row in early['candidates'] if row['offset_m']==chosen['offset_m'] and row['transition_m']==chosen['transition_m'])
    assert row['eligible'] and row['preview_maximum_steering_rad']<=.37
    assert not row['unknown_filled'] and chosen['speed_mps']<=.35
    assert not EntryConfig().prefer_earlier_transition
