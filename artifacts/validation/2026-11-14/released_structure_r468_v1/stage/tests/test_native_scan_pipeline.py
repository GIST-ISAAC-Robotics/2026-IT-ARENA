"""원천 배열이 부분 가림·추종·기록 단계까지 도달하는지 검사. 실제 주행과는 별도."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.scan_input import scan_angles,scan_times,scan_cell_widths,audit_scan_front,project_scan
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import mask_scan
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.traffic_recovery_trial import TimedPolicy
from arena_autonomy.follow_trace import snapshot_call,restore_call,freeze,replay_call,equal_output
from arena_autonomy.local_follow import LocalFollow
from test_local_follow import history,scan as uniform_scan


def native(n=508, first=.5, clockwise=True, gap=None):
    sign=-1 if clockwise else 1
    angles=tuple(sign*(-math.pi+i*math.tau/n+.0003*math.sin(i*.8)) for i in range(n))
    values=[8.]*n
    if gap is not None:
        values[int(np.argmin(abs(np.asarray(angles)-math.radians(gap))))]=math.inf
    return MeasurementScan(tuple(values),angles,tuple(first+i*.1/n for i in range(n)),
                           time_basis='simulation_acquisition')


@pytest.mark.parametrize('n',[424,500,508,524])
@pytest.mark.parametrize('clockwise',[True,False])
def test_front_audit_unknown_mask_and_variable_width(n,clockwise):
    s=native(n,clockwise=clockwise)
    assert audit_scan_front(s).reason=='ok'
    missing=native(n,clockwise=clockwise,gap=70)
    assert audit_scan_front(missing).unknown_samples==1
    assert audit_scan_front(missing).reason=='front_unobserved'
    assert audit_scan_front(missing,metadata_only=True).reason=='ok'
    masked=mask_scan(s,{'rear_half_angle_deg':30.,'sectors_deg':[[70,1.]]})
    assert masked.angles_rad==s.angles_rad and masked.acquired_at_s==s.acquired_at_s
    assert len(masked.ranges)==n and math.isnan(masked.ranges[0])
    assert len(scan_cell_widths(s))==n


def test_front_seam_hole_and_overlap_do_not_invent_observations():
    # 원천배열은 356도. 누락된4도가 정면이면 metadata 결측으로 분리한다.
    s=native();a=tuple(math.radians(2+i*356/507) for i in range(508))
    s=replace(s,angles_rad=a)
    assert audit_scan_front(s).reason=='front_unobserved'
    assert audit_scan_front(s).unknown_samples==0
    assert audit_scan_front(s,math.pi).reason=='ok' # 같은 공백이 보호 전방 밖으로 이동


@pytest.mark.parametrize('policy_type',[PartialObservationPolicy,TimedPolicy])
@pytest.mark.parametrize('clockwise',[False,True])
def test_partial_missing_uses_actual_order_and_times(policy_type,clockwise):
    p=policy_type();h=history();s=native(first=.4,clockwise=clockwise)
    points,ids,_=project_scan(s,h,.51,timing_verified=True)
    assert p.evaluate(s,points,h,.51,.4)['allowed']
    s=native(first=.5,clockwise=clockwise,gap=70)
    points,ids,_=project_scan(s,h,.61,timing_verified=True)
    result=p.evaluate(s,points,h,.61,.4)
    assert result['allowed'] and not result['unknown_filled']
    if isinstance(p,TimedPolicy):
        assert np.array_equal(p.current_times,scan_times(s)[ids])
    s=native(first=.6,clockwise=clockwise,gap=0)
    points,_,_=project_scan(s,h,.71,timing_verified=True)
    assert not p.evaluate(s,points,h,.71,.4)['allowed']


def test_uniform_access_and_projection_keep_original_result_exactly():
    s=uniform_scan(ranges=[8.]*500);h=history(.5,.4);now=.62
    before=h.project(s.ranges,s.angle_min,s.angle_increment,.05,12.,.5,.0002,now,'both',True)
    after=project_scan(s,h,now,timing_verified=True)
    assert np.array_equal(before[0],after[0]) and np.array_equal(before[1],after[1])
    assert before[2]==after[2]
    assert np.array_equal(scan_angles(s),s.angle_min+np.arange(500)*s.angle_increment)


@pytest.mark.parametrize('clockwise',[False,True])
def test_native_follow_and_trace_replay_without_uniform_metadata(clockwise):
    s=native(clockwise=clockwise);angles=np.asarray(s.angles_rad)
    # 장착 높이가 필요 없는 평면 회랑 ray cast. 실제 환경 정답은 제어 입력 아님.
    distances=np.minimum(.305/np.maximum(abs(np.sin(angles)),1e-12),
                         8./np.maximum(abs(np.cos(angles)),1e-12))
    s=replace(s,ranges_m=tuple(distances.tolist()))
    h=history();scene=motion_scene(s,h,.62,timing_verified=True,rear_half_angle_deg=0.)
    actual=LocalFollow().command(scene,h,0.,'left',0.,.02)
    captured=snapshot_call(s,h,.62,0,0.,'left',0.,.02,True,0.)
    _,restored,_=restore_call(captured,h.config)
    assert restored==s and not hasattr(restored,'angle_increment')
    replay=replay_call(LocalFollow(),captured,h.config)
    assert equal_output(freeze(actual),freeze(replay))
    assert actual[2]['reason']!='follow_input_stop'


def test_native_partial_follow_preserves_subnanosecond_first_stamp():
    from arena_autonomy.local_follow import FollowConfig
    follow=LocalFollow(FollowConfig(partial_observation_policy='bounded_local_v4',wall_offset_m=.305))
    h=history()
    outputs=[]
    for first in (np.nextafter(.4,1.),np.nextafter(.5,1.),np.nextafter(.6,1.)):
        s=native(first=float(first));angles=np.asarray(s.angles_rad)
        ranges=np.minimum(.305/np.maximum(abs(np.sin(angles)),1e-12),
                          8./np.maximum(abs(np.cos(angles)),1e-12))
        s=replace(s,ranges_m=tuple(ranges.tolist()))
        scene=motion_scene(s,h,first+.12,timing_verified=True,rear_half_angle_deg=0.)
        outputs.append(follow.command(scene,h,0.,'left',0.,.02,raw_scan=s))
        assert outputs[-1][2]['reason']!='follow_partial_scan_scene_mismatch'
    assert outputs[1][2]['reason']=='follow_release_wait'
    assert outputs[-1][0]>0., outputs[-1][2]


def test_native_trace_cost_profile_uses_raw_source_stamp(tmp_path):
    from arena_autonomy.follow_trace import FollowTrace
    from arena_autonomy.local_follow import FollowConfig
    from profile_follow_trace import profile_source,read_events
    f=LocalFollow(FollowConfig(wall_offset_m=.305))
    h=history();p=tmp_path/'native.jsonl';trace=FollowTrace(p,h.config,f.config)
    for first in (.4,.4,.5):
        s=native(first=first);angles=np.asarray(s.angles_rad)
        s=replace(s,ranges_m=tuple(np.minimum(.305/np.maximum(abs(np.sin(angles)),1e-12),
                  8./np.maximum(abs(np.cos(angles)),1e-12)).tolist()))
        now=first+.12;scene=motion_scene(s,h,now,timing_verified=True,rear_half_angle_deg=0.)
        output=f.command(scene,h,0.,'left',0.,.02)
        trace.record_call(s,h,now,0,0.,'left',0.,.02,True,0.,output)
    trace.close()
    result=profile_source(lambda:read_events(p),repeats=1,warmups=0)
    assert result['passed'] and result['baseline_matched']==3
    assert [row['new_source_stamp'] for row in result['rows']]==[True,False,True]
