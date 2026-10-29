import math
import time
import pytest
from arena_autonomy.follow_trace import snapshot_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_safety import LidarSafety
from recovery_policy import recovery_policy_class
from safety_shadow import ShadowSafety
from test_local_follow import scan, history


def sample(first=.5, now=.6, values=None, speed=0.):
    h = history(v=speed)
    raw = scan(first, [8.]*500 if values is None else values)
    return snapshot_call(raw,h,now,0,speed,'left',0.,.02,True,30.), h.config


def gate():
    return ShadowSafety(recovery_policy_class()(True,True,True),FollowConfig(),30.)


@pytest.mark.parametrize('command', [(0.,0.),(.35,.2),(0.,.2)])
def test_command_bound_and_explicit_zero(command):
    g=gate(); d,c=sample(); output,report=g.step(d,c,command)
    assert 0<=output[0]<=command[0]
    assert output[0]==command[0]
    assert output[1]==command[1]


@pytest.mark.parametrize('fault',['old_scan','nan_command','reverse_command','all_unknown',
                                  'close_obstacle','reverse_feedback','wrong_frame','missing_motion',
                                  'unverified_timing'])
def test_recovery_policy_still_stops_bad_input(fault):
    g=gate(); d,c=sample(); command=[.35,0.]
    if fault=='old_scan': d,c=sample(first=.1)
    if fault=='nan_command': command[0]=math.nan
    if fault=='reverse_command': command[0]=-.1
    if fault=='all_unknown': d,c=sample(values=[math.inf]*500)
    if fault=='close_obstacle': d,c=sample(values=[.06]*500)
    if fault=='reverse_feedback': d,c=sample(speed=-.1)
    if fault=='wrong_frame': d['scan']['frame_id']='incorrect_frame'
    if fault=='missing_motion': d['history']['wheels']={'times':[],'values':[]}
    if fault=='unverified_timing': d['timing_verified']=False
    output,report=g.step(d,c,command)
    assert output[0]==0


@pytest.mark.parametrize('fault',['command_transport','scan_transport','command_stamp'])
def test_delivery_assumption_is_separate_from_actual_gate(fault):
    g=gate(); d,c=sample(); assert g.step(d,c,[.35,0.])[0][0]>0
    if fault=='command_transport':g.node.command_wall=time.monotonic()-4
    if fault=='scan_transport':g.node.scan_wall=time.monotonic()-4
    if fault=='command_stamp':g.node.command.header.stamp.nanosec=0
    LidarSafety.control(g.node)
    assert g.output[0]==0 and g.report['reason']=='stale_input'


def test_separate_policy_and_history_per_gate():
    left,right=gate(),gate();d,c=sample()
    left.step(d,c,[.35,0.]);right.step(d,c,[.35,0.])
    assert left.node.partial_policy is not right.node.partial_policy
    assert left.motion.history is not right.motion.history
    left.node.partial_policy._latch('test_only',.6)
    assert left.node.partial_policy.latched and not right.node.partial_policy.latched


def test_clock_reversal_clears_independent_inputs():
    g=gate();d,c=sample();g.step(d,c,[.35,0.])
    d,c=sample(.3,.4);out,report=g.step(d,c,[.35,0.])
    assert out[0]==0 and report['reason']=='missing_input'
    assert g.node.scan is None and g.node.command is None
