"""실제 ROS 노드 생성과 기존 출력 감시의 각도 설정을 확인한다. 장치/주행 없음."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
import rclpy
from arena_vehicle_interface.actuation_ros import GuardedSimActuator

def probe(value):
    args = [] if value is None else ['--ros-args', '-p', f'max_steering_angle_rad:={value}']
    rclpy.init(args=args)
    node = None
    try:
        node = GuardedSimActuator()
        expected = .45 if value is None else value
        assert node.guard.max_angle == expected
        assert node.guard.timeout_us == 100_000
        assert node.guard.blocked
        return {'parameter': value, 'guard_max_angle': node.guard.max_angle,
                'initially_blocked': node.guard.blocked, 'timeout_us': node.guard.timeout_us}
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

rows = [probe(x) for x in (None, .2, .4578436017036438)]
destination = Path(__file__).with_name('ros_parameter_result.json')
with destination.open('x', encoding='utf8') as stream:
    json.dump({'passed': True, 'cases': rows, 'simulation_run': False,
               'physical_actuation': False}, stream, indent=2)
print(json.dumps({'passed': True, 'cases': len(rows)}))
