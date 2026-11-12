"""실제 ROS 생성·선택 옵션·기록·RESET·정상 종료. 센서/차량 구동 없음."""
import time
from types import SimpleNamespace

import pytest
rclpy = pytest.importorskip('rclpy')
from std_srvs.srv import Trigger
from arena_autonomy.local_pursuit import LocalPursuit


@pytest.mark.parametrize('enabled', [False, True])
def test_node_observer_option_only_changes_diagnostics(enabled):
    args = ['--ros-args', '-p', 'use_sim_time:=true', '-p', 'vision_execution:=process',
            '-p', f'vehicle_marker_observer_enabled:={str(enabled).lower()}',
            '-p', 'vehicle_marker_ids:="12,13"', '-r', '/drive:=/r340_test/drive',
            '-r', '/autonomy/status:=/r340_test/status']
    rclpy.init(args=args)
    node = None
    try:
        node = LocalPursuit()
        observer = node.vision_pipeline.vehicle_observer
        assert (observer is not None) == enabled
        sent, states = [], []
        node.publisher = SimpleNamespace(publish=sent.append)
        node.status_publisher = SimpleNamespace(publish=states.append)
        if enabled:
            observer.update(0., [dict(id=12, center_x=.5, center_y=.6, edge_fraction=.1,
                                    ambiguous=False, duplicates=1)], (720, 1280))
        node.control()
        assert sent[-1].drive.speed == 0  # 관측은 출발 허가가 아니다.
        import json
        status = json.loads(states[-1].data)
        assert ('vehicle_markers' in status) == enabled
        if enabled:
            assert status['vehicle_markers']['controls_speed'] is False
        node.on_reset(Trigger.Request(), Trigger.Response())
        if enabled:
            assert observer.stamp is None
        process = node.vision_pipeline.worker.process
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    assert not process.is_alive() and process.exitcode == 0
