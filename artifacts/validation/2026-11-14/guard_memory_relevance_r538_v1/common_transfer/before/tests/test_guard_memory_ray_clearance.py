"""현재 실제 광선의 옛 위치 통과만 검사한다. 현장 실측/폐루프 주행은 아니다."""
import copy
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.guard_memory_command import current_location_clearance,limited_memory_command
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import project_scan
from test_guard_memory_command import seeded,check,PARTIAL,guard_node
import test_independent_guard_route as fixture


def test_finite_rays_past_old_point_restore_cap_with_original_evidence_unchanged():
    p,h=seeded(((.400022,-.066134),));raw=fixture.scan(ranges=[2.]*500)
    old=p.guard_memory.original.copy();evidence=copy.deepcopy(p.latch_evidence)
    result=check(p,h,raw)
    assert result['accepted'] and result['speed_cap_mps']==.8
    assert result['reason']=='retained_memory_all_currently_observed'
    assert result['active_points_reobserved']==0 and result['active_points_ray_cleared']==1
    assert result['normal_cap_restored'] and result['all_retained_points_currently_resolved']
    assert not result['all_retained_points_reobserved'] and p.latched
    assert p.latch_evidence==evidence and len(p.guard_memory.points)==0
    np.testing.assert_array_equal(p.guard_memory.original,old)


@pytest.mark.parametrize('case',['unknown','occluded','insufficient_depth','near_21mm','over_range','behind_forward_ray'])
def test_unconfirmed_locations_remain_active(case):
    point=(-.4,0.) if case=='behind_forward_ray' else (.4,0.)
    p,h=seeded((point,));raw=fixture.scan(ranges=[2.]*500)
    if case=='unknown':raw.ranges[250]=float('nan')
    elif case=='occluded':raw.ranges[250]=.2
    elif case=='insufficient_depth':raw.ranges[250]=.485
    elif case=='near_21mm':raw.ranges[250]=.361
    elif case=='over_range':raw.ranges[250]=raw.range_max+.1
    else:
        raw.ranges[:125]=[float('nan')]*125
        raw.ranges[375:]=[float('nan')]*125
    before=np.array(raw.ranges)
    result=check(p,h,raw)
    before[375]=float('nan')  # frozen r528 check()의 명시 측방 결측
    assert len(p.guard_memory.points)==1 and result['active_points_ray_cleared']==0
    assert not result.get('normal_cap_restored',False)
    assert not result['accepted'] or result['speed_cap_mps']<=.1
    np.testing.assert_array_equal(np.array(raw.ranges),before)
    assert p.latched


def test_one_unobserved_point_retains_crawl_after_other_point_is_ray_cleared():
    p,h=seeded(((.4,-.066),(.06,.15)));result=check(p,h)
    assert result['accepted'] and result['speed_cap_mps']==.1
    assert result['active_points_ray_cleared']==1 and result['retained_points']==1
    assert not result['normal_cap_restored'] and len(p.guard_memory.original)==2


def test_current_raw_obstacle_stops_after_old_location_is_cleared():
    p,h=seeded(((.4,-.066),));raw=fixture.scan(ranges=[2.]*500);raw.ranges[250]=.05
    result=check(p,h,raw)
    assert result['active_points_ray_cleared']==1
    assert not result['accepted'] and result['current_raw_clearance_m']==0.
    assert p.latched


def test_masked_ray_and_scene_point_mismatch_do_not_handoff():
    raw=fixture.scan(ranges=[2.]*500);h=fixture.history()
    scene=motion_scene(raw,h,.6,timing_verified=True,rear_half_angle_deg=0.)
    rays=list(scene.rays);rays[250]=replace(rays[250],status='masked',hit_xy_m=None)
    scene=replace(scene,rays=tuple(rays))
    ids=np.array([r.source_index for r in scene.rays if r.status=='return'])
    points=np.asarray([r.hit_xy_m for r in scene.rays if r.status=='return'])
    clear,report=current_location_clearance(np.array([[.4,0.]]),scene,points,ids)
    assert not clear.any() and not report['accepted']
    points[0]+=.01
    clear,report=current_location_clearance(np.array([[.4,0.]]),scene,points,ids)
    assert not clear.any() and report['reason']=='current_ray_point_mismatch'


def test_continuously_carried_memory_can_clear_after_original_probe_and_buffer_expire():
    h=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=.5))
    p,_=seeded(((.3,.2),),h=h);old=p.guard_memory.original.copy();last=-1
    for tick in range(6,24):
        now=tick/10
        for sample in range(last+1,round(now*100)+1):
            h.add_mean_wheel(sample/100,.1/.0325);h.add_gyro(sample/100,0.)
        last=round(now*100)
        raw=fixture.scan(now-.1,ranges=[2.]*500)
        if tick<23:raw.ranges[250:380]=[float('nan')]*130
        result=check(p,h,raw,now=now,measured=.1)
        assert result['accepted']
        if tick<23:
            assert result['speed_cap_mps']==.1 and result['retained_points']==1
            np.testing.assert_allclose(p.guard_memory.points,old-[.1*(now-.4),0.],atol=1e-12)
        else:
            assert result['speed_cap_mps']==.8 and result['active_points_ray_cleared']==1
    assert h.wheels.times[0]>.4 and p.guard_memory.original_reference==.4
    np.testing.assert_array_equal(p.guard_memory.original,old)


def test_stop_does_not_become_positive_when_ray_clearance_succeeds():
    p,h=seeded(((.4,-.066),));result=check(p,h,requested=0.)
    assert not result['accepted'] and result['speed_cap_mps']==0.
    assert result['active_points_ray_cleared']==1 and p.latched


def test_actual_guard_control_uses_ray_handoff_and_preserves_raw_obstacle_stop():
    for obstacle in (False,True):
        p,h=seeded(((.4,-.066),));raw=fixture.scan(ranges=[2.]*500);raw.ranges[375]=float('nan')
        if obstacle:raw.ranges[250]=.05
        n,outputs,_=guard_node(raw,p,h)
        control,_=fixture.control_method();control(n)
        assert outputs[-1].drive.speed==(0. if obstacle else .8)
        assert p.latched and p.guard_memory.ray_cleared_total==1


def test_same_scan_skips_ray_work_but_transports_memory_and_checks_raw(monkeypatch):
    import arena_autonomy.guard_memory_command as module
    p,h=seeded();raw=fixture.scan(ranges=[2.]*500)
    assert check(p,h,raw)['speed_cap_mps']==.1
    before=p.guard_memory.reference
    def unexpected(*args,**kwargs):raise AssertionError('same scan reconstructed')
    monkeypatch.setattr(module,'motion_scene',unexpected)
    result=check(p,h,raw,now=.62)
    assert result['accepted'] and result['current_ray_clearance']['reason']=='same_scan_no_new_ray_evidence'
    assert p.guard_memory.reference==.62>before
    raw.ranges[250]=.05  # 同 source라도 현재 raw의 충돌 검사는 생략하지 않는다.
    result=check(p,h,raw,now=.64)
    assert not result['accepted'] and result['current_raw_clearance_m']==0.
