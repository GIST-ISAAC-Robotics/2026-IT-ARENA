from copy import deepcopy
import pytest
from m2_queue_eval import queue_window


def inputs():
    times=[i/100 for i in range(101)]
    trace=[{'t':i/10,'safe_speed_mps':0.,'autonomy':{'behavior':'HOLD'}} for i in range(11)]
    ego=[(t,0.,0.,0.) for t in times]
    near=[(t,1.,0.,0.) for t in times]
    far=[(t,2.+.35*t,0.,0.) for t in times]
    return [trace,ego,near,far]


def evaluate(data):
    return queue_window(*data,.2,.9,.18)


def test_far_moves_but_queue_holds():
    assert evaluate(inputs())['passed']


@pytest.mark.parametrize('stream',[1,2])
def test_body_motion_is_not_hidden_by_zero_command(stream):
    data=inputs()
    data[stream]=[(t,x+.02*t,y,a) for t,x,y,a in data[stream]]
    assert not evaluate(data)['passed']


def test_far_must_actually_move():
    data=inputs()
    data[3]=deepcopy(data[2])
    assert not evaluate(data)['gates']['far_moved']


@pytest.mark.parametrize('stream',[1,2,3])
def test_pose_gaps_fail(stream):
    data=inputs()
    data[stream]=[r for r in data[stream] if not .4<r[0]<.7]
    assert not evaluate(data)['passed']


def test_positive_safe_command_fails_even_if_pose_stopped():
    data=inputs()
    data[0][5]['safe_speed_mps']=.1
    assert not evaluate(data)['gates']['safe_zero_and_hold']


def test_trace_gap_or_missing_boundary_fails():
    data=inputs()
    data[0]=[r for r in data[0] if not .3<r['t']<.7]
    assert not evaluate(data)['gates']['trace_covered']
    data=inputs()
    data[0]=data[0][3:]
    assert not evaluate(data)['gates']['trace_covered']
