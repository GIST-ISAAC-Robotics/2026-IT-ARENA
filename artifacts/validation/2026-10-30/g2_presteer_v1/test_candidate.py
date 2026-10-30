from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[3]/'scripts')]
from candidate import SteeringRecoveryGuide
from arena_autonomy.bypass_entry_trial import EntryGuide
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from probe_bypass_entry import sensor_scene


def subject():
    h=SteeringRecoveryGuide(stationary_outer_boundary=True)
    h.path=np.array([[-.0675,0.],[1.,.2]]);h.offset=.24;h.transition=1.2;h.side='left';h.wall_confirmations=2
    h.last_requested_speed=0.
    history=MotionHistory(MotionConfig())
    for at in (.46,.48,.5):history.add_mean_wheel(at,0.)
    scene=SimpleNamespace(t_ref_s=.5,source_first_s=.4,occupied_endpoints_xy_m=((.5,.3),))
    old=dict(reason='no_entry_command_candidate',selected=None,guide_mode='measured_motion_transport_current_outer_wall',
        candidates=[dict(reason='known_occupied_command_hold',steering_rad=-.05)],reference=dict(walls={'left':{}}))
    return h,scene,history,old


def blocked(*args):
    return dict(eligible=False,steering_rad=args[5],target_mps=0.,clearance_m=.04,reason='no_braking_space')


def ready_only_after_angle_change(*args):
    previous,steer=args[4:6]
    eligible=previous==steer and steer>=0.
    return dict(eligible=eligible,steering_rad=steer,target_mps=.1 if eligible else 0.,clearance_m=.2)


def patches(old,angle):
    return (patch.object(EntryGuide,'evaluate',return_value=old),patch('candidate.source_valid',return_value=True),
        patch('candidate.current_outer_wall_padding',return_value=(np.array([.055]),{})),
        patch.object(SteeringRecoveryGuide,'_angle',side_effect=angle),
        patch('candidate.entry_motion_region',return_value={}),patch('candidate.region_blocked',return_value=None))


def call(h,sc,history,old,angle=ready_only_after_angle_change,speed=0.):
    from contextlib import ExitStack
    with ExitStack() as stack:
        for p in patches(old,angle):stack.enter_context(p)
        return h.evaluate(sc,history,speed,-.05,.02)


def test_future_angle_can_only_request_zero_now():
    h,sc,hi,old=subject();r=call(h,sc,hi,old)
    assert r['selected']['speed_mps']==0.
    assert abs(r['selected']['steering_rad']+.05)<=.06+1e-12
    assert r['steering_recovery']['kind']=='zero_speed_presteer'
    assert not r['steering_recovery']['future_permission_remembered']


@pytest.mark.parametrize('fault',['previous_positive','one_nonzero','current_nonzero','old_sample','single_sample'])
def test_motor_feedback_and_previous_zero_are_required(fault):
    h,sc,hi,old=subject();speed=0.
    if fault=='previous_positive':h.last_requested_speed=.1
    elif fault=='current_nonzero':speed=.01
    else:
        hi=MotionHistory(MotionConfig())
        values={'one_nonzero':[(.48,.01),(.5,0.)], 'old_sample':[(.40,0.),(.42,0.)],
                'single_sample':[(.5,0.)]}[fault]
        for t,v in values:hi.add_mean_wheel(t,v)
    r=call(h,sc,hi,old,speed=speed)
    assert r['selected'] is None


def test_new_block_after_presteer_does_not_receive_cached_permission():
    h,sc,hi,old=subject();assert call(h,sc,hi,old)['selected']['speed_mps']==0.
    r=call(h,sc,hi,old,blocked)
    assert r['selected'] is None


def test_existing_dynamic_overlap_is_not_exempted():
    h,sc,hi,old=subject();sc.occupied_endpoints_xy_m=((0.,.12),)
    r=call(h,sc,hi,old,lambda *args:(_ for _ in ()).throw(AssertionError('do not search occupied body')))
    assert r is old and r['selected'] is None


@pytest.mark.parametrize('fault',['already_positive','rejoin','unknown','no_wall','fault_output'])
def test_valid_original_or_unavailable_context_is_unchanged(fault):
    h,sc,hi,old=subject()
    if fault=='already_positive':old['selected']=dict(speed_mps=.35,steering_rad=-.05)
    elif fault=='rejoin':h.phase='rejoin'
    elif fault=='unknown':old['candidates'][0]['reason']='large_motion_region_unobserved'
    elif fault=='no_wall':h.wall_confirmations=1
    else:old['guide_mode']='entry_guide_input_reset';old['candidates']=[]
    assert call(h,sc,hi,old,blocked) is old


def test_actual_front_points_still_block_alternative_arc():
    h=SteeringRecoveryGuide(stationary_outer_boundary=True)
    scene,_=sensor_scene(targets=((.18,0.),))
    points=np.asarray(scene.occupied_endpoints_xy_m)
    r=h._angle(scene,points,np.full(len(points),.055),0.,0.,.06,.02)
    assert not r['eligible'] and r['target_mps']==0.


def test_reset_does_not_preserve_zero_readiness():
    h,sc,hi,old=subject();h.reset()
    assert h.last_requested_speed is None and h.path is None
