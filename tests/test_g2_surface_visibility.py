"""기하 통과가 관측 허가로 승격되지 않는지와 평가 입력의 경계 검사."""
import math
import numpy as np
import pytest

from g0_geometry import TruthBox
from g2_visibility_probe import sequential_scene
from g2_surface_visibility import inspect_stage, validate_trace, recent_frames
from arena_autonomy.lateral_intent import LateralIntentConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, support


def straight_rows():
    return [dict(t_s=i*.02,rear_x_m=-.0675+.4*i*.02,rear_y_m=0.,yaw_rad=0.,
                 applied_speed_mps=.4,applied_steering_rad=0.) for i in range(200)]


def test_default_scene_keeps_previous_input_and_explicit_parameters_match():
    rows=straight_rows()
    assert sequential_scene(rows,.4,.12)==sequential_scene(rows,.4,.12,target_y=.24,rear_half_angle_deg=30.)
    changed=sequential_scene(rows,.4,.12,target_y=.15)[0]
    original=sequential_scene(rows,.4,.12)[0]
    assert original.source_first_s==changed.source_first_s
    assert original.source_last_s==changed.source_last_s
    assert any(a.hit_xy_m!=b.hit_xy_m for a,b in zip(original.rays,changed.rays))


def test_full_view_counterfactual_still_measures_real_occluder():
    scene,labels,_=sequential_scene(straight_rows(),.4,.12,target_y=.15,rear_half_angle_deg=0.,
                                    extra_boxes=(TruthBox(.7,0.,.2,label='occluder'),))
    assert not any(r.status=='masked' for r in scene.rays)
    assert sum(l=='occluder' for l in labels)>0
    assert len(scene.rays)==500
    assert scene.source_last_s-scene.source_first_s==pytest.approx(.0998)


def test_low_target_absence_is_not_clearance_authorization():
    result=inspect_stage(straight_rows(),.4,height=.06)
    assert result['target_returns']==0
    assert not result['driving_authorized']
    assert result['truth_transport_evaluation_only']
    assert any(c['kind']=='stored_future_path' for c in result['checks'])


def test_history_uses_oldest_acquisition_not_only_reference_time():
    rows=straight_rows()
    _,_,ref=sequential_scene(rows,.52,.12,target_y=.15)
    frames=recent_frames(rows,.52,.12,30.,ref)
    assert len(frames)==2
    assert all(.52-s.source_first_s < .3 for s,_ in frames)
    assert max(.52-s.source_first_s for s,_ in frames)==pytest.approx(.2998)


def test_current_occluder_is_not_overridden_by_old_clear_scene():
    rows=straight_rows()
    old,_,old_ref=sequential_scene(rows,.3,.12,target_y=.15,rear_half_angle_deg=0.)
    current,_,ref=sequential_scene(rows,.4,.12,target_y=.15,rear_half_angle_deg=0.,
                                  extra_boxes=(TruthBox(.65,0.,.2,label='occluder'),))
    from g2_visibility_probe import local_pose
    cfg=LateralIntentConfig(surface_policy='two_wheel_grass')
    q=np.array([[.85,0.]])
    current_only=support(current,[],q,cfg,ExecutionConfig())
    recent=support(current,[(old,local_pose(old_ref,ref))],q,cfg,ExecutionConfig())
    assert not current_only['supported']
    assert not recent['supported']
    assert recent['history_supported']==0


@pytest.mark.parametrize('kind',['time','nan','bool','speed','steering','empty'])
def test_invalid_trace_rejected_before_evaluation(kind):
    rows=straight_rows()
    if kind=='time': rows[3]['t_s']+=.001
    elif kind=='nan': rows[3]['yaw_rad']=math.nan
    elif kind=='bool': rows[3]['rear_x_m']=True
    elif kind=='speed': rows[3]['applied_speed_mps']=.81
    elif kind=='steering': rows[3]['applied_steering_rad']=.38
    else: rows=[]
    with pytest.raises(ValueError): validate_trace(rows)
