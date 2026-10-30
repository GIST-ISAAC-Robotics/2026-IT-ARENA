"""현재 증거·시간/출처 변화가 준비 대기 기억보다 우선하는지 검사한다."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
import candidate
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig


def execution():
    return HandoffExecutionConfig(entry_policy='corridor_local_v1',preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True,reacquire_policy='known_hold_v1',
        bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'))


@pytest.fixture
def setup(monkeypatch):
    obj=candidate.PreparationHoldFollow(execution=execution(),allow_simulation_trial=True)
    obj.monitor.motion_evidence=lambda:False
    patch=np.array([[1.43,-.02],[1.43,0.],[1.43,.02]])
    obj.preparation_anchor=dict(key=('lidar',0,10.),time=10.14,patch=patch)
    scene=NS(source_frame_id='lidar',source_epoch=0,source_first_s=10.5,t_ref_s=10.64)
    monkeypatch.setattr(candidate,'front_patch',lambda *_:(patch,'current_front_patch'))
    monkeypatch.setattr(candidate,'transport_points',lambda points,*_:points)
    monkeypatch.setattr(BypassHandoffFollow,'_follow_entry_space',lambda self,scene,normal,previous,target:
        (target,dict(reason='two_wall_straight_reference_unavailable',applied=False)))
    return obj,scene,patch


def test_known_preparation_can_only_reduce_command(setup):
    obj,scene,_=setup
    target,detail=obj._follow_entry_space(scene,(1.,0.,{}),0.,1.)
    assert target==0. and detail['current_front_evidence'] and detail['bypass_permission'] is False
    assert detail['free_space_assumed'] is False


@pytest.mark.parametrize('change',[dict(source_epoch=1),dict(source_frame_id='other'),dict(source_first_s=9.9),
    dict(t_ref_s=11.1401,source_first_s=11.),dict(t_ref_s=10.1,source_first_s=10.),dict(t_ref_s=10.8)])
def test_new_epoch_reverse_time_staleness_or_expiry_releases(setup,change):
    obj,scene,_=setup
    for key,value in change.items():setattr(scene,key,value)
    assert obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)[0]==.4
    assert obj.preparation_anchor is None


def test_moving_or_replaced_front_is_not_retained(setup):
    obj,scene,patch=setup
    obj.monitor.motion_evidence=lambda:True
    assert obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)[0]==.4
    obj.monitor.motion_evidence=lambda:False
    obj.preparation_anchor=dict(key=('lidar',0,10.),time=10.14,patch=patch+np.array([.10,0.]))
    assert obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)[0]==.4


def test_no_memory_or_path_outside_patch_keeps_original(setup,monkeypatch):
    obj,scene,_=setup
    monkeypatch.setattr(BypassHandoffFollow,'_follow_entry_space',lambda *args:(.2,dict(reason='patch_outside_g1_swept_arc')))
    assert obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)[0]==.2
    obj.reset()
    assert obj.preparation_anchor is None


def test_reused_scan_does_not_extend_anchor(setup,monkeypatch):
    obj,scene,patch=setup
    output=(0.,0.,dict(reason='handoff_follow',preparation=dict(applied=True),entry_space=dict(hold=True)))
    monkeypatch.setattr(BypassHandoffFollow,'command',lambda *a,**kw:output)
    obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())
    armed=obj.preparation_anchor['time'];scene.t_ref_s+=.10
    obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())
    assert obj.preparation_anchor['time']==armed


@pytest.mark.parametrize('reason',['handoff_reverse_motion_hold','handoff_input_fault',
    'handoff_invalid_input','handoff_source_reset'])
def test_input_failure_clears_preparation_memory(setup,monkeypatch,reason):
    obj,scene,_=setup
    monkeypatch.setattr(BypassHandoffFollow,'command',lambda *a,**kw:(0.,0.,dict(reason=reason)))
    assert obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())[0]==0.
    assert obj.preparation_anchor is None


@pytest.mark.parametrize('state',['BYPASS','REACQUIRE'])
def test_memory_does_not_override_admitted_bypass(setup,monkeypatch,state):
    obj,scene,_=setup;obj.mode=state
    output=(.25,.03,dict(reason='handoff_bypass'))
    monkeypatch.setattr(BypassHandoffFollow,'command',lambda *a,**kw:output)
    assert obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())==output
    assert obj.preparation_anchor is None


def test_actual_encoder_motion_compensation_and_missing_history(setup,monkeypatch):
    from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
    from arena_autonomy.scene_motion import transport_points
    obj,scene,old=setup
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,history_s=3.))
    for t in np.arange(9.9,10.7,.01):
        history.add_mean_wheel(float(t),.35/.0325);history.add_gyro(float(t),0.)
    obj.current_history=history
    # 전진0.175 m이면 정지 물체의 현재 좌표는 그만큼 가까워진다. 정답 자세는 쓰지 않는다.
    current=old-np.array([.175,0.])
    monkeypatch.setattr(candidate,'front_patch',lambda *_:(current,'current_front_patch'))
    monkeypatch.setattr(candidate,'transport_points',transport_points)
    speed,detail=obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)
    assert speed==0. and detail['compensated_patch_displacement_m']<1e-8
    obj.current_history=MotionHistory(MotionConfig())
    assert obj._follow_entry_space(scene,(.4,0.,{}),0.,.4)[0]==.4
    assert obj.preparation_anchor is None
