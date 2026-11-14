"""실제 launch_ros 인수 생성 + rclpy 초기화만 검사. root/runner 전용, Windows에서는 실행 금지.

원 launcher의 IncludeLaunchDescription은 제외하여 Gazebo/제어기/장치를 시작하지 않는다.
OFF/ON을 한 명령에서 각각 한 번 검사하며 child는 parameter 선언 후 바로 종료한다.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys


def child(output, expected_profile, ros_args):
    import rclpy
    from rclpy.node import Node
    node = None
    initialized = False
    try:
        rclpy.init(args=ros_args)
        initialized = True
        node = Node('r559_optional_parameter_probe')
        profile = node.declare_parameter('vehicle_marker_profile_path', '').value
        observer = node.declare_parameter('vehicle_marker_observer_enabled', True).value
        vision = node.declare_parameter('vision_execution', 'unset').value
        assert type(profile) is str and profile == expected_profile
        assert type(observer) is bool and observer is False
        assert vision == 'process'
        record = dict(passed=True, profile=profile, observer=observer, vision=vision,
            actual_ros_args=ros_args, publisher_created=False, subscription_created=False)
        with output.open('x', encoding='utf-8') as stream:
            json.dump(record, stream, indent=2)
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()


def parent(output_dir):
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
    from launch_ros.actions import Node

    output_dir.mkdir(parents=True, exist_ok=False)
    source = Path(__file__).with_name('marker_course.launch.py')
    spec = importlib.util.spec_from_file_location('r559_marker_launcher', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    results = []
    for case, profile in [('off', ''), ('on', 'config/perception/vehicle_marker_approx_b0390.json')]:
        actions = module.generate_launch_description().entities
        assert sum(isinstance(a, IncludeLaunchDescription) for a in actions) == 1
        actions = [a for a in actions if not isinstance(a, IncludeLaunchDescription)]
        output = output_dir / (case+'.json')
        probe = Node(executable=sys.executable, arguments=[str(Path(__file__).resolve()),
            '--child', '--output', str(output.resolve()), '--expected-profile', profile], output='screen')
        service = LaunchService(argv=[])
        service.include_launch_description(LaunchDescription([
            SetLaunchConfiguration('r537_marker_profile', profile), *actions, probe]))
        code = service.run()
        assert code == 0 and output.is_file(), (case, code)
        result = json.loads(output.read_text(encoding='utf-8'))
        assert result['passed']
        assert not any(item == 'vehicle_marker_profile_path:=' for item in result['actual_ros_args'])
        results.append(dict(case=case, **result))
    (output_dir / 'result.json').write_text(json.dumps(dict(passed=True, cases=results,
        scope='native launch_ros transport and rclpy parser only; no include/Gazebo/control/device node'), indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=True, cases=len(results))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--child', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--expected-profile', default='')
    args, ros_args = parser.parse_known_args()
    if args.child:
        assert args.output is not None
        child(args.output, args.expected_profile, ros_args)
    else:
        assert args.output_dir is not None and not ros_args
        parent(args.output_dir)
