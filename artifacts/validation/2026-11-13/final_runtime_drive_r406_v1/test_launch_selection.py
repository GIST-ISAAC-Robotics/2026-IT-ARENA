"""실제 launch 치환만 확인한다. simulation OpaqueFunction/Node/process는 실행하지 않는다."""
import importlib.util
from pathlib import Path

import pytest
from prepare import A, verify_prepared


@pytest.mark.parametrize('selected', [False, True])
def test_common_launch_declarations_keep_explicit_runtime_selection(selected):
    from launch import LaunchContext, LaunchDescription
    from launch.actions import IncludeLaunchDescription
    before = verify_prepared()
    context = LaunchContext()
    if selected:
        context.launch_configurations.update(g1_runtime_policy='cad_runtime_r377_v1',
            g1_current_corner_budget='r396_v1')
    spec = importlib.util.spec_from_file_location('r406_launch_declarations', A/'course.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    found = []

    def walk(action, simulation=False):
        if isinstance(action, LaunchDescription):
            children = action.entities
        elif isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            is_sim = action.launch_description_source.location.endswith('/simulation.launch.py')
            for child in action.execute(context) or []:
                walk(child, simulation or is_sim)
            if is_sim:
                found.append({key: context.launch_configurations[key]
                    for key in ('g1_runtime_policy', 'g1_current_corner_budget')})
            return
        else:
            kind = type(action).__name__
            if simulation and kind not in ('DeclareLaunchArgument', 'SetLaunchConfiguration'):
                return
            assert kind in {'GroupAction', 'PushLaunchConfigurations', 'PopLaunchConfigurations',
                'PushEnvironment', 'PopEnvironment', 'SetLaunchConfiguration', 'DeclareLaunchArgument',
                'SetParameter', 'OpaqueFunction', 'ResetEnvironment', 'SetEnvironmentVariable'}, kind
            children = action.execute(context) or []
        for child in children:
            walk(child, simulation)

    walk(module.generate_launch_description())
    assert found == [dict(g1_runtime_policy='cad_runtime_r377_v1' if selected else 'off',
                          g1_current_corner_budget='r396_v1' if selected else 'off')]
    assert verify_prepared() == before
