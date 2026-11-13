"""실제 launch 치환만 확인한다. simulation OpaqueFunction/Node/process는 실행하지 않는다."""
import importlib.util
import ast
import json
from pathlib import Path

import pytest
from prepare import A, ROOT, SIM, read, verify_prepared


@pytest.mark.parametrize('selected', [False, True])
def test_common_launch_declarations_keep_explicit_runtime_selection(selected):
    from launch import LaunchContext, LaunchDescription
    from launch.actions import IncludeLaunchDescription
    before = verify_prepared()
    context = LaunchContext()
    if selected:
        context.launch_configurations.update(g1_runtime_policy='cad_runtime_r377_v1',
            g1_current_corner_budget='r396_v1', g1_crop_edge_component_trial='true')
    spec = importlib.util.spec_from_file_location('r418_launch_declarations', A/'course.launch.py')
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
                    for key in ('g1_runtime_policy', 'g1_current_corner_budget', 'g1_crop_edge_component_trial')})
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
                          g1_current_corner_budget='r396_v1' if selected else 'off',
                          g1_crop_edge_component_trial='true' if selected else 'false')]
    assert verify_prepared() == before


@pytest.mark.parametrize('selected', [False, True])
def test_same_resolved_flag_reaches_both_node_parameter_dicts(selected):
    # _setup/Node는 실행하지 않는다. 실제 양쪽 Node가 받는 두 표현식과 그 원천만 치환한다.
    from launch import LaunchContext
    source = (ROOT/SIM).read_text(encoding='utf8')
    tree = ast.parse(source)
    name = 'g1_crop_edge_component_trial'
    parameter_values = [value for node in ast.walk(tree) if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values)
        if isinstance(key, ast.Constant) and key.value == name]
    assert len(parameter_values) == 2
    assert all(isinstance(value, ast.Name) and value.id == 'crop_edge' for value in parameter_values)
    assigns = [node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'crop_edge' for target in node.targets)]
    assert len(assigns) == 1
    spec = importlib.util.spec_from_file_location('r418_parameter_expression', ROOT/SIM)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    context = LaunchContext(); context.launch_configurations[name] = 'true' if selected else 'false'
    namespace = {**module.__dict__, 'context': context}
    value = eval(compile(ast.Expression(assigns[0]), SIM, 'eval'), namespace)
    assert value is selected
    namespace['crop_edge'] = value
    assert [eval(compile(ast.Expression(node), SIM, 'eval'), namespace) for node in parameter_values] == [selected, selected]


def test_actual_followtrace_carries_selected_field_and_other_header_unchanged(tmp_path):
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
    from arena_autonomy.follow_trace import FollowTrace
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.local_follow import FollowConfig
    expected = read(A/'expected_header.json')
    assert FollowConfig().crop_edge_component_trial is False
    execution = dict(expected['execution_config'])
    execution['bypass'] = BypassExecutionConfig(**execution['bypass'])
    target = tmp_path/'trace.jsonl'
    trace = FollowTrace(target, MotionConfig(**expected['motion_config']), FollowConfig(**expected['follow_config']),
        mode='g1_g2_handoff_trial', execution_config=HandoffExecutionConfig(**execution), current_corner_budget='r396_v1')
    trace.close()
    rows = [json.loads(line) for line in target.read_text(encoding='utf8').splitlines()]
    assert rows[0]['data'] == expected
    assert rows[-1]['kind'] == 'footer' and rows[-1]['data']['capture_errors'] == 0
