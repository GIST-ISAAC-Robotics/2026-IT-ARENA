"""실제 ROS launch 치환으로 G1→pursuit→demo→simulation 경계를 확인한다.

simulation 본문·Node·프로세스는 실행하지 않는다.
"""
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch
import ament_index_python.packages
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

ROOT = Path(__file__).resolve().parents[4]
SHARE = ROOT / 'src/arena_bringup'
def resolve(rate=None):
    original = ament_index_python.packages.get_package_share_directory
    with patch('ament_index_python.packages.get_package_share_directory', side_effect=lambda name:
               str(SHARE) if name == 'arena_bringup' else original(name)):
        return _resolve(rate)


def _resolve(rate=None):
    context = LaunchContext()
    if rate is not None:
        context.launch_configurations['autonomy_control_rate_hz'] = str(rate)
    entry = SHARE / 'launch/g1_follow.launch.py'
    spec = importlib.util.spec_from_file_location('g1_rate_entry', entry)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = []
    def string(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    def walk(action):
        if isinstance(action, LaunchDescription):
            children = action.entities
        elif isinstance(action, IncludeLaunchDescription):
            # Resolve/load declarations only; the simulation's OpaqueFunction is never executed.
            action.launch_description_source.get_launch_description(context)
            location = action.launch_description_source.location
            if location.endswith('/simulation.launch.py'):
                result.append({string(k): string(v) for k,v in action.launch_arguments})
                return
            children = action.execute(context) or []
        else:
            # Allow only launch setup actions in this diagnostic, never process execution.
            allowed = {'GroupAction', 'PushLaunchConfigurations', 'PopLaunchConfigurations',
                       'PushEnvironment', 'PopEnvironment', 'SetLaunchConfiguration',
                       'DeclareLaunchArgument', 'SetParameter', 'OpaqueFunction',
                       'ResetEnvironment', 'SetEnvironmentVariable'}
            assert type(action).__name__ in allowed, type(action).__name__
            children = action.execute(context) or []
        for child in children:
            walk(child)
    walk(module.generate_launch_description())
    assert len(result) == 1
    return result[0]


if __name__ == '__main__':
    default, selected = resolve(), resolve(10.)
    differences = {key: [default.get(key), selected.get(key)] for key in default.keys() | selected.keys()
                   if default.get(key) != selected.get(key)}
    report = dict(default_hz=default['autonomy_control_rate_hz'],
                  selected_hz=selected['autonomy_control_rate_hz'], differences=differences,
                  simulation_started=False)
    report['passed'] = (float(report['default_hz']) == 50. and float(report['selected_hz']) == 10.
                        and set(differences) == {'autonomy_control_rate_hz'})
    print(json.dumps(report))
    raise SystemExit(0 if report['passed'] else 1)
