"""Windows launcher 동작 대역 검사. native ROS CLI parser 검사는 별도 1회 인계한다."""
import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from arena_autonomy.vehicle_marker_channel import MarkerChannel

A = Path(__file__).resolve().parent


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.launch_configurations[self.name]


class ParameterValue:
    def __init__(self, value, *, value_type):
        self.value, self.value_type = value, value_type


class SetParameter:
    def __init__(self, *, name, value):
        self.name, self.value = name, value


class OpaqueFunction:
    def __init__(self, *, function):
        self.function = function


def generate(filename='marker_course.launch.py'):
    tree = ast.parse((A / filename).read_text(encoding='utf-8'))
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    scope = dict(__file__=str(A / filename), Path=Path, LaunchConfiguration=LaunchConfiguration,
        ParameterValue=ParameterValue, SetParameter=SetParameter, OpaqueFunction=OpaqueFunction,
        LaunchDescription=lambda rows:rows,
        DeclareLaunchArgument=lambda name,default_value:N(name=name,default=default_value),
        IncludeLaunchDescription=lambda source:N(include=source), PythonLaunchDescriptionSource=lambda source:source)
    exec(compile(tree, filename, 'exec'), scope)
    return scope['generate_launch_description']()


def evaluate(actions, **overrides):
    context = N(launch_configurations=dict(overrides))
    parameters = {}
    for action in actions:
        if isinstance(action, OpaqueFunction):
            nested = action.function(context)
        elif isinstance(action, SetParameter):
            nested = [action]
        elif hasattr(action, 'default'):
            context.launch_configurations.setdefault(action.name, action.default)
            continue
        else:
            continue  # IncludeLaunchDescription을 실행하지 않는다.
        for item in nested:
            assert isinstance(item, SetParameter)
            value = item.value
            if isinstance(value, ParameterValue):
                assert value.value_type is str
                value = value.value.perform(context) if isinstance(value.value, LaunchConfiguration) else value.value
                assert isinstance(value, str)
            parameters[item.name] = value
    return parameters


def test_before_reproduces_recorded_unconditional_empty_override():
    values = evaluate(generate('marker_course.launch.py.before'))
    assert 'vehicle_marker_profile_path' in values
    assert f"vehicle_marker_profile_path:={values['vehicle_marker_profile_path']}" == 'vehicle_marker_profile_path:='


@pytest.mark.parametrize('overrides', [{}, {'r537_marker_profile':''}])
def test_off_omits_only_empty_optional_profile(overrides):
    values = evaluate(generate(), **overrides)
    assert values == dict(vehicle_marker_observer_enabled=False, vision_execution='process')
    assert type(values['vehicle_marker_observer_enabled']) is bool


@pytest.mark.parametrize('profile', ['config/perception/vehicle_marker_approx_b0390.json',
    '/tmp/marker profile.json', '/tmp/marker\'s "profile".json', 'false'])
def test_on_preserves_exact_string_without_boolean_or_shell_coercion(profile):
    values = evaluate(generate(), r537_marker_profile=profile)
    assert values == dict(vehicle_marker_profile_path=profile, vehicle_marker_observer_enabled=False, vision_execution='process')
    assert type(values['vehicle_marker_profile_path']) is str
    assert type(values['vehicle_marker_observer_enabled']) is bool


def test_marker_consumer_default_off_does_not_create_publishers_or_subscriptions():
    class Node:
        def declare_parameter(self, name, default):
            assert (name, default) == ('vehicle_marker_profile_path', '')
            return N(value=default)

        def get_parameter(self, name):
            assert name == 'use_sim_time'
            return N(value=False)

        def create_publisher(self, *args):
            raise AssertionError('OFF must not create publisher')

        def create_subscription(self, *args):
            raise AssertionError('OFF must not create subscription')

    for producer in (False, True):
        channel = MarkerChannel(Node(), producer=producer)
        assert channel.profile is None and channel.snapshot(0.) is None


def test_include_and_nonoptional_parameter_contract_unchanged():
    before, after = generate('marker_course.launch.py.before'), generate()
    assert before[-1].include == after[-1].include
    assert [(a.name,a.value) for a in before if isinstance(a,SetParameter) and a.name!='vehicle_marker_profile_path'] == [
        (a.name,a.value) for a in after if isinstance(a,SetParameter)]
