import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.lidar_motion import MotionUnavailable


def node_double():
    commands, statuses = [], []
    command = AckermannDriveStamped()
    command.header.stamp.sec = 10
    command.drive.speed = 2.
    scan = LaserScan(angle_min=-math.pi, angle_increment=2*math.pi/500,
                     range_min=.05, range_max=12., ranges=[2.]*500)
    scan.header.stamp.sec = 10
    def project(_):
        x = np.linspace(-1, 3, 60)
        return np.vstack((np.column_stack((x, x*0+.425)), np.column_stack((x, x*0-.425)))), 120
    node = SimpleNamespace(command=command, scan=scan, scan_wall=time.monotonic(), command_wall=time.monotonic(),
                           wall_timeout=3., limit=.37, last_clock=None, wheelbase=.145, length=.20, width=.15,
                           motion=SimpleNamespace(project=project),
                           publisher=SimpleNamespace(publish=commands.append), status=SimpleNamespace(publish=statuses.append),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000,
                             to_msg=lambda: command.header.stamp)))
    return node, commands, statuses


def test_clear_path_keeps_speed():
    node, commands, statuses = node_double()
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 2.


def test_delayed_old_scan_stops_even_with_fresh_transport_and_running_command():
    node, commands, statuses = node_double()
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 2.
    # 첫 점 취득 10.0 + 한 바퀴 0.1 + 추가 전송 0.3초.
    # 방금 도착했어도 첫 점은 이미 400 ms 전 값이다. 명령은 새 값으로 유지한다.
    node.command.header.stamp.nanosec = 400_000_000
    node.scan_wall = node.command_wall = time.monotonic()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_400_000_000, to_msg=lambda: node.command.header.stamp))
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 0.
    assert json.loads(statuses[-1].data)['reason'] == 'stale_input'


@pytest.mark.parametrize('fault', ['no_scan', 'no_command', 'old_command', 'old_scan', 'nan_command',
                                   'reverse', 'unknown_front', 'motion_gap', 'transport'])
def test_faults_stop_independent_gate(fault):
    node, commands, statuses = node_double()
    if fault == 'no_scan': node.scan = None
    if fault == 'no_command': node.command = None
    if fault == 'old_command': node.command.header.stamp.sec = 9
    if fault == 'old_scan': node.scan.header.stamp.sec = 9
    if fault == 'nan_command': node.command.drive.speed = math.nan
    if fault == 'reverse': node.command.drive.speed = -1.
    if fault == 'unknown_front': node.scan.ranges[250] = math.nan
    if fault == 'transport': node.scan_wall -= 4.
    if fault == 'motion_gap':
        def fail(_): raise MotionUnavailable('motion_gap')
        node.motion.project = fail
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 0.
    assert json.loads(statuses[-1].data)['reason'] != 'clear'


def test_obstacle_inside_footprint_stops():
    node, commands, _ = node_double()
    node.motion.project = lambda _: (np.tile([.1, 0.], (40,1)), 40)
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 0.


def test_clock_rollback_clears_inputs_and_motion_history():
    node, commands, statuses = node_double()
    resets = []
    node.last_clock = 11.
    node.motion.history = SimpleNamespace(reset=lambda: resets.append(True))
    LidarSafety.control(node)
    assert resets == [True] and node.scan is None and node.command is None
    assert commands[-1].drive.speed == 0.
    assert json.loads(statuses[-1].data)['reason'] == 'missing_input'
