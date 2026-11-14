"""기존 실제 벽 근거의 정지차 실행 연결. 이동차 도형 반례는 공유 구현을 검사한다."""
from dataclasses import replace
import json
import pytest
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_entry_trial import EntryGuide
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.moving_pass import rear_wall_witness as moving_witness
from arena_autonomy.wall_reference_support import rear_wall_witness
from test_moving_pass import moving_controller
from test_structure_route_speed import sectors


def test_default_preserved_and_current_structure_input_required():
    assert not BypassExecutionConfig().rear_wall_witness
    assert not EntryGuide().rear_wall_witness_enabled
    assert moving_witness is rear_wall_witness
    with pytest.raises(ValueError): BypassExecutionConfig(rear_wall_witness=1)
    with pytest.raises(ValueError,match='known-structure'):
        EntryGuide(rear_wall_witness=True,tracked_wall_fit=True,wall_reanchor=True,stationary_outer_boundary=True)


def test_handoff_guide_replacement_retains_explicit_witness():
    base=moving_controller()
    config=replace(base.config,known_structure_policy='structure_aware_v1',
                   known_structure_sectors_json=json.dumps(sectors(.06)))
    options=replace(base.execution,bypass=replace(base.execution.bypass,rear_wall_witness=True))
    ctrl=BypassHandoffFollow(config,options,allow_simulation_trial=True)
    assert ctrl.bypass.guide.rear_wall_witness_enabled
    assert len(ctrl.bypass.guide.witness_sectors)==4
    ctrl.reset()
    assert ctrl.bypass.guide.rear_wall_witness_enabled
    assert ctrl.bypass.guide.raw_scan is None


def test_ros_evaluator_requires_explicit_handoff_context():
    from g1_handoff_eval import launch_arguments
    launch=dict(actuation_mode='virtual_mcu',autonomy_mode='local_pursuit',speed_bump=False,
        lidar_rate_hz=10.,lidar_compensation='both',lidar_acquisition='sequential',
        speed_profile='g1_open_1p5',control_rate_hz=50.)
    with pytest.raises(ValueError,match='wall witness'):
        launch_arguments('g1g2','paced',launch,True,wall_witness=True)
    args=launch_arguments('g1g2','paced',launch,True,wall_witness=True,observed_motion=True)
    assert 'evaluation_handoff_wall_witness:=true' in args
