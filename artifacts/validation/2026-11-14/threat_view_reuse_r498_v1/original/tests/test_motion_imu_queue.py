"""큐 지연 뒤에도 취득 시각·입력 경계·만료 기준을 바꾸지 않는지 확인한다."""
import pytest
from rclpy.qos import qos_profile_sensor_data

from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import relative_poses
from test_lidar_motion_ros import NodeDouble, imu


class QosNode(NodeDouble):
    def __init__(self):
        super().__init__()
        self.qos = {}

    def create_subscription(self, kind, topic, callback, qos):
        self.qos[topic] = qos
        return super().create_subscription(kind, topic, callback, qos)


def delayed_batch(now=1.2, boundary=None):
    node = NodeDouble()
    node.t = now
    adapter = MotionInput(node)
    if boundary is not None:
        adapter.begin_input_boundary(boundary)
    for i in range(40):
        stamp = 1.+i*.005
        adapter.on_imu(imu(stamp, z=1.))
        if boundary is None or stamp >= boundary:
            adapter.history.add_mean_wheel(stamp, 0.)
    return adapter


def test_history_qos_is_separate_from_global_sensor_profile():
    node = QosNode()
    MotionInput(node)
    qos = node.qos['/camera/imu']
    assert qos.depth == 64 and qos_profile_sensor_data.depth == 5
    assert qos is not qos_profile_sensor_data
    for name in ('history','reliability','durability','deadline','lifespan',
                 'liveliness','liveliness_lease_duration','avoid_ros_namespace_conventions'):
        assert getattr(qos, name) == getattr(qos_profile_sensor_data, name)


def test_delayed_batch_retains_original_time_and_integrated_yaw():
    adapter = delayed_batch()
    assert adapter.counts['gyro'] == 40 and adapter.counts['rejected'] == 0
    assert adapter.history.gyro.times[0] == pytest.approx(1.)
    assert adapter.history.gyro.times[-1] == pytest.approx(1.195)
    assert relative_poses(adapter.history, [1.], 1.195)[0,2] == pytest.approx(-.195)


def test_queue_does_not_make_an_entirely_old_batch_current():
    adapter = delayed_batch(now=1.4)
    with pytest.raises(MotionUnavailable):
        relative_poses(adapter.history, [1.1], 1.4)


def test_reset_boundary_rejects_old_queued_samples():
    adapter = delayed_batch(boundary=1.1)
    assert adapter.counts['gyro'] == 20
    assert adapter.gyro_rejections == {'gyro_before_boundary': 20}
    assert adapter.history.gyro.times[0] == pytest.approx(1.1)
    assert adapter.counts['resets'] == 1
