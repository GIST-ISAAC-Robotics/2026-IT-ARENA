"""추정값을 검증 완료로 바꾸지 않는 명시 비구동 시간 정책."""
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from arena_autonomy.follow_trace import snapshot_call, restore_call, replay_call, equal_output
from arena_autonomy.lidar_motion import MotionUnavailable, MotionHistory
from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.scene_motion import measurement_motion_scene, motion_scene
from arena_autonomy.scan_input import project_scan
from arena_autonomy.local_follow import LocalFollow
from test_native_measurement_motion import scan, history
from test_lidar_motion_ros import NodeDouble


POLICY='host_estimated_v1'


def test_explicit_estimate_contract_preserves_geometry_and_provenance():
    original=scan();estimated=replace(original,time_basis='unverified_estimate');h=history()
    before=measurement_motion_scene(original,h,1.12,timing_verified=True)
    after=measurement_motion_scene(estimated,h,1.12,timing_verified=False,timing_policy=POLICY)
    assert before==after  # 같은 수치로 계약만 비교하며 실물 정확도 검증은 아니다.
    assert estimated.time_basis=='unverified_estimate'
    _,_,meta=project_scan(estimated,h,1.12,timing_policy=POLICY)
    assert meta['hardware_timing_verified'] is False and meta['timing_policy']==POLICY
    assert meta['time_basis']=='unverified_estimate'


@pytest.mark.parametrize('fault',['future','stale','frame','no_encoder','no_gyro','wrong_policy','wrong_basis'])
def test_estimate_permission_does_not_mask_missing_or_invalid_inputs(fault):
    s=scan(time_basis='unverified_estimate');h=history();now=1.12;policy=POLICY
    if fault=='future':now=1.09
    elif fault=='stale':now=1.4
    elif fault=='frame':s=replace(s,source_frame_id='other')
    elif fault=='no_encoder':h.wheels.times.clear();h.wheels.values.clear()
    elif fault=='no_gyro':h.gyro.times.clear();h.gyro.values.clear()
    elif fault=='wrong_policy':policy='true'
    elif fault=='wrong_basis':s=replace(s,time_basis='simulation_acquisition')
    with pytest.raises(MotionUnavailable):
        measurement_motion_scene(s,h,now,timing_verified=False,timing_policy=policy)


def test_unknown_returns_remain_unknown_under_estimate_permission():
    values=[2.]*508;values[20]=math.nan
    s=scan(time_basis='unverified_estimate',ranges_m=tuple(values))
    scene=measurement_motion_scene(s,history(),1.12,timing_verified=False,timing_policy=POLICY)
    assert scene.rays[20].status=='unknown' and scene.rays[20].hit_xy_m is None


def test_trace_roundtrip_keeps_false_verified_and_explicit_selection():
    s=scan(time_basis='unverified_estimate');h=history()
    data=snapshot_call(s,h,1.12,0,1.,'left',0.,.02,False,30.,timing_policy=POLICY)
    restored,same,_=restore_call(data,h.config)
    assert restored['timing_verified'] is False and restored['timing_policy']==POLICY and same==s
    expected=LocalFollow().command(measurement_motion_scene(s,h,1.12,timing_verified=False,
        timing_policy=POLICY),h,1.,'left',0.,.02)
    assert equal_output(expected,replay_call(LocalFollow(),data,h.config))
    del data['timing_policy']
    rejected=replay_call(LocalFollow(),data,h.config)
    assert rejected[2]['input_error']=='scan_timing_unverified'


def test_default_snapshot_has_no_new_policy_key():
    data=snapshot_call(scan(),history(),1.12,0,1.,'left',0.,.02,True,30.)
    assert 'timing_policy' not in data


@pytest.mark.parametrize('fault',['environment','sim_clock','verified','mode'])
def test_ros_estimate_policy_is_only_allowed_in_explicit_no_drive_environment(fault):
    node=NodeDouble(dict(motion_scan_time_policy=POLICY,lidar_compensation='both',use_sim_time=False))
    node.execution_environment='hardware_shadow_v1'
    if fault=='environment':node.execution_environment='default'
    elif fault=='sim_clock':node.parameters['use_sim_time']=True
    elif fault=='verified':node.parameters['motion_scan_timing_verified']=True
    elif fault=='mode':node.parameters['lidar_compensation']='none'
    with pytest.raises(ValueError):MotionInput(node)
    assert not node.subscriptions


def test_ros_estimate_policy_accepts_only_genuine_history_without_fake_zero():
    node=NodeDouble(dict(motion_scan_time_policy=POLICY,lidar_compensation='both',use_sim_time=False))
    node.execution_environment='hardware_shadow_v1';node.t=1.12
    adapter=MotionInput(node)
    assert not adapter.verified and adapter.timing_policy==POLICY
    with pytest.raises(MotionUnavailable):adapter.project(scan(time_basis='unverified_estimate'))
    assert adapter.history.wheels.times==[] and adapter.counts['drive_motor']==0


@pytest.mark.parametrize('scope',['solo_baseline','solo_or_guard_parallel','absent_guard_baseline',
                                 'released_baseline','present_guard_baseline','released_parallel'])
def test_current_recheck_keeps_same_result_for_explicit_estimate_contract(scope):
    import copy
    from arena_autonomy import current_path_recheck
    from test_current_path_recheck import fixture_for
    args=fixture_for(scope)
    before=current_path_recheck.rebase(*args)
    old,current=copy.deepcopy(args[1]),copy.deepcopy(args[2])
    for item in (old,current):
        item['timing_verified']=False;item['timing_policy']=POLICY
        item['scan']['measurements']['time_basis']='unverified_estimate'
    _,raw,h=restore_call(current,args[4].config)
    scene=motion_scene(raw,h,current['reference_s'],timing_verified=False,
        epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    after=current_path_recheck.rebase(args[0],old,current,scene,h,args[5])
    assert equal_output(before,after)
