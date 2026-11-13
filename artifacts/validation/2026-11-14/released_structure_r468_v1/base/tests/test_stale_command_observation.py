"""명령 만료가 정상 센서 기억을 끊지 않되 구동/소실 기억 경계는 유지한다."""
import json
import time
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_lidar_safety import node_double
from test_partial_observation import sample
from test_local_follow import history

def stamp(t):return Time(sec=int(t),nanosec=round((t-int(t))*1e9))

@pytest.mark.parametrize('kind',['command_age','command_transport','scan_missing','known_disappearance'])
def test_valid_observation_and_command_lifetimes_are_independent(kind):
    node,commands,statuses=node_double()
    node.partial_policy=PartialObservationPolicy(allow_narrow_forward=True)
    node.motion.history=history()
    points=np.full((60,2),20.)
    node.motion.project=lambda raw:(points.copy(),len(points))
    node.command.drive.speed=.2
    for now in np.arange(.5,1.61,.1):
        now=round(float(now),6)
        node.get_clock=lambda t=now:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=round(t*1e9),to_msg=lambda:stamp(t)))
        node.command.header.stamp=stamp(.5 if now<1.6 else now)
        node.command_wall=time.monotonic()-(4. if kind=='command_transport' and .8<=now<1.6 else 0.)
        node.scan_wall=time.monotonic()
        node.scan=sample(now-.1,[] if now==.5 else [250])
        points[:]=20.
        if kind=='known_disappearance' and now<=.7:points[0]=[.4,0.]
        if kind=='scan_missing' and .8<=now<1.6:node.scan=sample(.4,[250])
        LidarSafety.control(node)
        if .8<=now<1.6:assert commands[-1].drive.speed==0.
    report=json.loads(statuses[-1].data)
    partial=report['partial_observation']
    assert report['reference_s']==1.6 and report['command_stale'] is False
    if kind in ('command_age','command_transport'):
        assert commands[-1].drive.speed==.2 and partial['latch_reason'] is None
    else:
        expected='partial_history_expired' if kind=='scan_missing' else 'previous_point_in_missing_sweep'
        assert commands[-1].drive.speed==0. and partial['latch_reason']==expected

@pytest.mark.parametrize('fault',['scan_wall','invalid_command','reverse_command','future_command','missing_command'])
def test_invalid_input_does_not_refresh_history(fault):
    node,commands,statuses=node_double()
    node.partial_policy=PartialObservationPolicy(allow_narrow_forward=True)
    node.motion.history=history()
    node.command.header.stamp=stamp(.1)
    node.scan=sample(.5,[250])
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=600_000_000,to_msg=lambda:stamp(.6)))
    if fault=='scan_wall':node.scan_wall-=4.
    elif fault=='invalid_command':node.command.drive.speed=float('nan')
    elif fault=='reverse_command':node.command.drive.speed=-1.
    elif fault=='future_command':node.command.header.stamp=stamp(.7)
    else:node.command=None
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.
    assert node.partial_policy.previous is None
