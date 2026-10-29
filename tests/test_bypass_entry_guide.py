"""진입 경로의 기억을 현재 관측/구동 허가로 바꾸지 않는지 검사한다."""
import numpy as np
import pytest
from arena_autonomy.bypass_entry_trial import EntryGuide, current_outer_wall_padding, observed_straight_wall
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.bypass_entry_trial import entry_motion_region, EntryConfig
from arena_autonomy.pass_release_trial import motion_region, lateral_config
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from probe_bypass_entry import sensor_scene


def stationary_history():
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for at in np.arange(.2,.701,.01):
        history.add_mean_wheel(float(at),0.)
        history.add_gyro(float(at),0.)
    return history


def started():
    guide=EntryGuide();history=stationary_history()
    scene,_=sensor_scene(targets=((1.2,0.),))
    out=guide.evaluate(scene,history,0.,0.,.02)
    assert out['selected'] and guide.path is not None
    return guide,history,out


def test_retained_guide_is_rechecked_without_remembered_permission():
    guide,history,first=started()
    before=guide.path.copy();offset=guide.offset
    scene,_=sensor_scene(targets=((1.2,0.),),now=.6,first=.5)
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['selected'] is not None and out['selected']['offset_m']==offset
    assert out['guide_mode']=='measured_motion_transport_current_outer_wall'
    assert not out['guide_permission_remembered'] and not out['control_authorized']
    # 정지 상태에서는 곡선을 다음 시점의 후륜축부터 재생성하지 않는다.
    np.testing.assert_allclose(guide.path,before,atol=1e-12)
    assert out['reference']['inner_points_still_occupied']


def test_new_near_obstacle_blocks_a_remembered_guide():
    guide,history,first=started()
    scene,_=sensor_scene(targets=((.18,0.),),now=.6,first=.5)
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['selected'] is None


def test_new_front_unobserved_region_cannot_reuse_old_permission():
    guide,history,first=started()
    scene,_=sensor_scene(now=.6,first=.5,missing=tuple(range(225,276)))
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['selected'] is None and not out['unknown_filled']


@pytest.mark.parametrize('kind',['epoch','stale','regression','missing_history'])
def test_motion_or_scene_discontinuity_clears_guide(kind):
    guide,history,first=started()
    kwargs=dict(now=.6,first=.5)
    if kind=='epoch':kwargs['epoch']=1
    elif kind=='stale':kwargs=dict(now=.71,first=.4)
    elif kind=='regression':kwargs=dict(now=.49,first=.39)
    else:history=MotionHistory(history.config)
    scene,_=sensor_scene(targets=((1.2,0.),),**kwargs)
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['selected'] is None and guide.path is None


def test_current_outer_wall_must_remain_available():
    guide,history,first=started()
    side=guide.side
    # 선택한 외곽 반평면을 실제 무반환으로 바꾼다. 과거 벽을 새 반환으로 쓰지 않는다.
    missing=tuple(range(250,500)) if side=='left' else tuple(range(0,250))
    scene,_=sensor_scene(now=.6,first=.5,missing=missing)
    out=guide.evaluate(scene,history,0.,first['selected']['steering_rad'],.02)
    assert out['selected'] is None
    assert out['reason']=='entry_guide_current_outer_wall_unavailable'


def test_fixed_outer_wall_assumption_retains_near_target_padding_and_every_point():
    cfg=FollowConfig();scene,_=sensor_scene(targets=((1.2,0.),))
    _,wall=observed_straight_wall(scene,cfg,'right')
    original=per_ray_padding(scene,.055,.5)
    padding,evidence=current_outer_wall_padding(scene,cfg,'right',wall)
    points=np.asarray(scene.occupied_endpoints_xy_m)
    assert len(padding)==len(points)==len(original)
    assert evidence['static_return_count']>=12
    assert all(span>=.65 for span in evidence['accepted_spans_m'])
    assert not evidence['static_object_identity_verified']
    target=abs(points[:,1])<.08
    assert np.any(target)
    np.testing.assert_array_equal(padding[target],original[target])
    assert np.min(padding)>=.055


def test_many_short_surfaces_are_not_merged_across_missing_rays():
    cfg=FollowConfig()
    # 모든 세 번째 광선을 지우면 긴 면 가정을 만들 수 없다.
    scene,_=sensor_scene(missing=tuple(range(0,500,3)))
    padding,evidence=current_outer_wall_padding(scene,cfg,'right',dict(slope=0.,intercept_m=0.))
    assert evidence['static_return_count']==0
    np.testing.assert_array_equal(padding,per_ray_padding(scene,.055,.5))


def test_small_aligned_vehicle_surface_is_not_a_long_static_wall():
    from dataclasses import replace
    cfg=FollowConfig();scene,_=sensor_scene()
    # 현재 외곽에서 차량 크기보다 긴 면을 관측하지 못한 조건.
    rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.status=='return' and
        (r.hit_xy_m[0]<-.1 or r.hit_xy_m[0]>.1) else r for r in scene.rays)
    partial=replace(scene,rays=rays)
    padding,evidence=current_outer_wall_padding(partial,cfg,'right',dict(slope=0.,intercept_m=0.))
    assert evidence['static_return_count']==0


def test_static_boundary_is_off_by_default_and_needs_distinct_scans():
    assert not EntryGuide().stationary_outer_boundary
    state=EntryGuide(stationary_outer_boundary=True);history=stationary_history()
    scene,_=sensor_scene(targets=((1.2,0.),))
    first=state.evaluate(scene,history,0.,0.,.02)
    assert state.start_confirmations==1 and first['selected'] is None
    again=state.evaluate(scene,history,0.,0.,.02)
    assert state.start_confirmations==1 and again['selected'] is None
    newer,_=sensor_scene(targets=((1.2,0.),),now=.6,first=.5)
    next_result=state.evaluate(newer,history,0.,0.,.02)
    assert state.wall_confirmations==2 and next_result['selected'] is not None
    active=state.evaluate(newer,history,0.,next_result['selected']['steering_rad'],.02)
    assert active['outer_boundary_padding']['static_return_count']>0


def test_same_current_wall_assumption_does_not_turn_missing_rays_into_free():
    cfg=FollowConfig();scene,_=sensor_scene(missing=tuple(range(225,276)))
    _,wall=observed_straight_wall(scene,cfg,'right')
    padding,detail=current_outer_wall_padding(scene,cfg,'right',wall)
    assert detail['static_return_count']>0
    region=entry_motion_region(scene,0.,.35,0.,0.,.02,cfg,lateral_config(cfg),padding)
    assert region['unobserved']>0


def test_no_boundary_assumption_uses_original_observation_result():
    cfg=FollowConfig();scene,_=sensor_scene(targets=((.4,0.),))
    args=(scene,.1,.2,.05,0.,.02,cfg,lateral_config(cfg))
    assert entry_motion_region(*args)==motion_region(*args)


def test_wider_candidate_is_explicit_and_not_default():
    assert max(map(abs,EntryConfig().offsets_m))==.24
    option=EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.27,.27))
    assert max(map(abs,option.offsets_m))==.27


def test_point_gap_ranking_is_not_a_future_free_space_claim():
    from arena_autonomy.bypass_entry_trial import preview_point_gap
    cfg=FollowConfig()
    x=np.linspace(-.0675,1.4,80);straight=np.c_[x,np.zeros_like(x)]
    closer=np.c_[x,np.full_like(x,.1)]
    points=np.array([[.7,.3]])
    assert preview_point_gap(closer,points,np.array([.055]),cfg)<preview_point_gap(straight,points,np.array([.055]),cfg)


def test_confirmed_boundary_still_does_not_allow_a_new_near_obstacle():
    state=EntryGuide(stationary_outer_boundary=True);history=stationary_history()
    a,_=sensor_scene(targets=((1.2,0.),));state.evaluate(a,history,0.,0.,.02)
    b,_=sensor_scene(targets=((1.2,0.),),now=.6,first=.5)
    active=state.evaluate(b,history,0.,0.,.02)
    assert active['selected'] is not None
    c,_=sensor_scene(targets=((.18,0.),),now=.7,first=.6)
    blocked=state.evaluate(c,history,0.,active['selected']['steering_rad'],.02)
    assert blocked['selected'] is None


def scene_at_side(x, y=.16):
    from probe_bypass_entry_sequence import render_scan
    from arena_autonomy.scene_motion import motion_scene
    history=stationary_history()
    physical=[dict(t=t,rear_pose=[x-.0675,y,0.],speed=0.,steering=0.) for t in (0.,.6)]
    scan=render_scan(physical,.6)
    return motion_scene(scan,history,.6,timing_verified=True,rear_half_angle_deg=30.)


def test_inner_band_blocks_observed_car_before_rejoin():
    guide=EntryGuide(rejoin_enabled=True);guide.side='left';guide.offset=.16
    scene=scene_at_side(1.2)
    fit,detail=observed_straight_wall(scene,guide.config,'left')
    band=guide._rejoin_band(scene,fit)
    assert not band['supported'] and band['unobserved']>0


def test_inner_band_requires_positive_observation_after_passing():
    guide=EntryGuide(rejoin_enabled=True);guide.side='left';guide.offset=.16
    scene=scene_at_side(2.)
    fit,detail=observed_straight_wall(scene,guide.config,'left')
    band=guide._rejoin_band(scene,fit)
    assert band['supported'] and band['behind_vehicle_not_certified']
    assert band['rear_unobserved']>0 and band['front_unobserved']==0
    assert band['actual_motion_still_requires_current_support']


def test_visible_rear_half_vehicle_still_blocks_candidate_trigger():
    guide=EntryGuide(rejoin_enabled=True);guide.side='left';guide.offset=.16
    scene=scene_at_side(1.4)
    fit,detail=observed_straight_wall(scene,guide.config,'left')
    band=guide._rejoin_band(scene,fit)
    assert not band['supported'] and band['current_occupied_returns']>0


def test_missing_inner_band_never_means_car_has_disappeared():
    from dataclasses import replace
    guide=EntryGuide(rejoin_enabled=True);guide.side='left';guide.offset=.16
    scene=scene_at_side(2.)
    rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.status=='return' and
        r.direction_xy[1]<0 else r for r in scene.rays)
    scene=replace(scene,rays=rays)
    fit,detail=observed_straight_wall(scene,guide.config,'left')
    assert not guide._rejoin_band(scene,fit)['supported']


def test_quantized_speed_above_output_cap_does_not_reset_retained_direction():
    guide,history,first=started()
    before=guide.path.copy(); side=guide.side; sign=guide.intent.sign
    scene,_=sensor_scene(targets=((1.2,0.),),now=.6,first=.5)
    out=guide.evaluate(scene,history,.364649147291673,first['selected']['steering_rad'],.02)
    assert out['reason']!='entry_guide_input_reset'
    assert guide.path is not None and guide.side==side and guide.intent.sign==sign
    np.testing.assert_allclose(guide.path,before,atol=1e-12)
    assert out['selected'] is None or out['selected']['speed_mps']<=.35
    # 범위를 벗어난 속도는 새 허가나 이전 경로 유지로 쓰지 않는다.
    bad=guide.evaluate(scene,history,1.01,first['selected']['steering_rad'],.02)
    assert bad['reason']=='entry_guide_input_reset' and guide.path is None
