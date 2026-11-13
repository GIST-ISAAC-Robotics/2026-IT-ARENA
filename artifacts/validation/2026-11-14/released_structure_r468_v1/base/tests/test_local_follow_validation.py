import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('g1_validation', ROOT/'scripts/validate_local_follow.py')
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


def test_hold_gate_requires_a_contiguous_observed_interval():
    interrupted = [{'t': i*.02, 'speed': 0., 'command': 0. if i % 2 == 0 else .1} for i in range(60)]
    assert validation.longest_hold_s(interrupted, 0., 2., .02) == 0.
    continuous = [{'t': i*.02, 'speed': 0., 'command': 0.} for i in range(21)]
    assert validation.longest_hold_s(continuous, 0., 2., .02) == pytest.approx(.4)


def test_actor_scenario_is_continuous_and_no_hidden_ego_dependency():
    assert validation.actor_x('stop_restart', 2.) == validation.actor_x('stop_restart', 5.) == pytest.approx(2.3)
    assert validation.actor_x('stop_restart', 6.) == pytest.approx(2.75)
    assert validation.actor_x('empty', 0.) is None


def test_g1_launch_is_explicit_opt_in_without_safety_remap():
    text = (ROOT/'src/arena_bringup/launch/g1_follow.launch.py').read_text()
    assert "SetParameter(name='g1_follow_enabled', value=True)" in text
    assert 'GroupAction(scoped=False' in text  # 종료 타이머의 launch 설정 문맥 보존
    assert '/drive/safe' not in text and 'remappings' not in text
    node = (ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text()
    assert "declare_parameter('g1_follow_enabled', False)" in node


def test_gazebo_validator_collects_latched_mcu_diagnostic_before_shutdown():
    import ast
    text = (ROOT/'scripts/validate_basic_autonomy.py').read_text()
    ast.parse(text)
    assert "'/actuation/diagnostics'" in text
    assert 'durability=DurabilityPolicy.TRANSIENT_LOCAL' in text
    assert "output/'actuation_diagnostics.json'" in text
    assert "if report['actuation_diagnostics']['control_deadline_samples']:" in text
