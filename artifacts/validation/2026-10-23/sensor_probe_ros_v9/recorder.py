"""기록기 소스/기본 옵션은 유지하고 이번 자식의 DDS 적용만 검사한다."""
from transport_support import configure, inspect
configure()
import rclpy
original_create = rclpy.create_node


def inspected_create(*args, **kwargs):
    node = original_create(*args, **kwargs)
    try:
        inspect(node)
    except BaseException:
        node.destroy_node()
        raise
    return node


rclpy.create_node = inspected_create
from record_sensor_bag import main
main()
