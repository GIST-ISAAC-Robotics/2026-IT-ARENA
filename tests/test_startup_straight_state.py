"""직진 관측기의 확인/초기화 반례. 경로 fixture이며 출발 성공 근거가 아니다."""
import numpy as np

from arena_autonomy.startup_straight_candidate import StartupStraightCandidate
from arena_autonomy.local_scene import scene_from_scan
from probe_grid_follow import stationary_ranges, stationary_history

SENSOR=dict(xyz_m=[.06,0.,.095],rate_hz=10.,masked_rear_half_angle_deg=30.)
ROOM=np.array([[[-2.,-.425],[3.,-.425]],[[3.,-.425],[3.,.425]],[[3.,.425],[-2.,.425]],[[-2.,.425],[-2.,-.425]]])


def scene(first=.5,repeat=0,epoch=0,fault=False):
    values,_=stationary_ranges(dict(x=0.,y=0.,yaw_rad=0.),[],ROOM,SENSOR,500,.05,12.)
    if fault:values[250]=np.nan
    stamps=first+np.arange(500)*.0002
    return scene_from_scan(values.tolist(),angle_min=-np.pi,angle_increment=2*np.pi/500,
        range_min=.05,range_max=12.,acquired_at_s=stamps.tolist(),t_ref_s=float(stamps[-1]+.002+.02*repeat),
        source_epoch=epoch,sensor_xy_m=(.06,0.))


def baseline(reason='follow_gap_hold'):
    return (0.,0.,dict(reason=reason,boundary_age_s=.1018,
                      path_points=np.column_stack((np.linspace(-.25,1.8,36),np.zeros(36))).tolist()))


def test_distinct_scans_confirm_while_duplicates_do_not():
    h=stationary_history(.135,SENSOR)
    c=StartupStraightCandidate(enabled=True,rounded_occupancy=True)
    for repeat in range(5):
        result=c.evaluate(scene(repeat=repeat),h,baseline(),0.)
        assert result['reason']=='distinct_scan_confirmation_wait' and result['confirmations']==1
    result=c.evaluate(scene(.6),h,baseline(),0.)
    assert result['selected'] and result['confirmations']==2
    assert 0<result['proposed_speed_mps']<=.2 and result['proposed_steering_rad']==0.
    assert not result['driving_authorized'] and not result['runtime_connected']


def test_fault_drops_confirmation_and_needs_new_scans_after_recovery():
    h=stationary_history(.135,SENSOR)
    c=StartupStraightCandidate(enabled=True,rounded_occupancy=True)
    c.evaluate(scene(),h,baseline(),0.)
    assert c.evaluate(scene(.6),h,baseline(),0.)['selected']
    assert c.evaluate(scene(.7,fault=True),h,baseline(),0.)['reason']=='front_unobserved'
    result=c.evaluate(scene(.8),h,baseline(),0.)
    assert not result['selected'] and result['confirmations']==1
    assert c.evaluate(scene(.9),h,baseline(),0.)['selected']


def test_stop_positive_baseline_and_epoch_reset_do_not_leak_selection():
    h=stationary_history(.135,SENSOR)
    c=StartupStraightCandidate(enabled=True,rounded_occupancy=True)
    c.evaluate(scene(),h,baseline(),0.)
    assert c.evaluate(scene(.6),h,baseline(),0.)['selected']
    assert not c.evaluate(scene(.7),h,baseline('STOP'),0.)['selected']
    assert not c.evaluate(scene(.8),h,(.1,0.,baseline()[2]),0.)['selected']
    assert not c.evaluate(scene(.9),h,baseline(),0.)['selected']
    assert c.evaluate(scene(1.),h,baseline(),0.)['selected']
    result=c.evaluate(scene(1.1,epoch=1),h,baseline(),0.)
    assert not result['selected'] and result['confirmations']==1


def test_time_reversal_does_not_keep_prior_confirmations():
    h=stationary_history(.135,SENSOR)
    c=StartupStraightCandidate(enabled=True,rounded_occupancy=True)
    c.evaluate(scene(.6),h,baseline(),0.)
    assert c.evaluate(scene(.7),h,baseline(),0.)['selected']
    assert not c.evaluate(scene(.5),h,baseline(),0.)['selected']
