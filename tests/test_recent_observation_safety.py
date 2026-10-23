import json
import time
from types import SimpleNamespace

import pytest
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.observation_support import RecentScenes
from arena_autonomy.scene_motion import motion_scene
from test_lidar_safety import node_double
from test_local_follow import history, scan
from test_recent_observation_follow import config


def candidate_node():
    node, commands, statuses = node_double()
    node.recent_observation_enabled = True
    node.observation_cache = RecentScenes()
    node.observation_config = config()
    node.observation_epoch = 0
    node.previous_safe_steering = 0.
    node.rear_blind_half_angle = 30.
    node.wheelbase = .135
    clock_stamp = node.command.header.stamp
    node.get_clock = lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=600_000_000,
                        to_msg=lambda:clock_stamp))
    node.command.header.stamp.sec = 0
    node.command.header.stamp.nanosec = 600_000_000
    node.command.drive.speed = .8
    node.scan = scan(.5, [2.]*500)
    node.scan.ranges[375] = float('inf')
    h = history()
    def project(raw):
        observation = motion_scene(raw, h, .6, timing_verified=True)
        return observation.occupied_endpoints_xy_m, len(observation.occupied_endpoints_xy_m)
    node.motion = SimpleNamespace(project=project,history=h,verified=True)
    return node,commands,statuses


def test_independent_scoped_gate_caps_side_unknown_without_filling():
    node,commands,statuses = candidate_node()
    LidarSafety.control(node)
    assert commands[-1].drive.speed == pytest.approx(.2)
    report = json.loads(statuses[-1].data)
    assert report['recent_observation']['supported']
    assert report['recent_observation']['unknown_filled'] is False
    assert report['recent_observation']['rear_dynamic_risk_unresolved'] is True


@pytest.mark.parametrize('fault', ['front_unknown','near_return','stale_scan','stale_command','wall_timeout',
                                   'negative_feedback','missing_feedback','same_scan_expired'])
def test_independent_faults_still_stop(fault):
    node,commands,statuses = candidate_node()
    if fault=='front_unknown':node.scan.ranges[250]=float('inf')
    if fault=='near_return':node.scan.ranges[250]=.16
    if fault=='stale_scan':node.scan=scan(.2,[2.]*500)
    if fault=='stale_command':node.command.header.stamp.nanosec=0
    if fault=='wall_timeout':node.scan_wall=time.monotonic()-4.
    if fault=='negative_feedback':node.motion.history=history(v=-.1)
    if fault=='missing_feedback':node.motion.history=history(end=.2)
    if fault=='same_scan_expired':
        LidarSafety.control(node)
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=900_000_000,
                            to_msg=lambda:node.command.header.stamp))
        node.command.header.stamp.nanosec=900_000_000
    LidarSafety.control(node)
    assert commands[-1].drive.speed == 0.


def test_zero_command_cannot_be_revived_by_supported_scene():
    node,commands,statuses = candidate_node()
    node.command.drive.speed=0.
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.


def test_clock_reversal_invalidates_cached_evidence():
    node,commands,statuses = candidate_node()
    LidarSafety.control(node)
    assert node.observation_cache.scenes
    node.last_clock=1.
    LidarSafety.control(node)
    assert not node.observation_cache.scenes and node.observation_epoch==1
    assert commands[-1].drive.speed==0.
