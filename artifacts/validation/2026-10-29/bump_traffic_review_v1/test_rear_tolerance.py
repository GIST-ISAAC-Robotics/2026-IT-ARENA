"""진행 연결 전 민감도 도구: 폭만 바뀌며 실제 점유/정면/원시 고장은 그대로 거절한다."""
import math
import numpy as np
import pytest
from probe_rear_tolerance import sensitivity
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history


def make(*, x=-.047, y=.0765, value=math.inf, neighbor=4., count=2, sign=1):
    # 점의 방향을 둘러싼 실제 광선 쌍을 사용한다. 임의 ray 번호에 맞춰 점을 움직이지 않는다.
    raw=scan(.5,[4.]*500)
    angle=math.atan2(sign*y, x-.06)
    index=math.floor((angle-raw.angle_min)/raw.angle_increment)
    for i in range(index,index+count):raw.ranges[i]=value
    raw.ranges[index-1]=neighbor
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    return scene,raw,[[x,sign*y]],np.full(500,.10),.2,.15


@pytest.mark.parametrize('sign',[-1,1])
def test_limits_only_change_registered_rear_strip(sign):
    args=make(sign=sign)
    a=sensitivity(*args,.001);b=sensitivity(*args,.003)
    assert a['reason']=='outside_rear_fringe'
    assert b['accepted'] and b['unknown_filled'] is False
    assert b['residual_hidden_object_risk'] is True


@pytest.mark.parametrize('x,y',[(.001,.0765),(-.101,.0765),(-.047,.075),(-.047,.0851)])
def test_forward_bumper_body_and_large_excursion_still_excluded(x,y):
    assert sensitivity(*make(x=x,y=y),.010)['reason']=='outside_rear_fringe'


@pytest.mark.parametrize('bad',[math.nan,-math.inf,.02,-1.])
def test_near_or_corrupt_nonreturn_not_accepted(bad):
    assert not sensitivity(*make(value=bad),.010)['accepted']


@pytest.mark.parametrize('near',[.08,.19,.4])
def test_actual_occupied_or_near_neighbor_not_overridden(near):
    assert not sensitivity(*make(neighbor=near),.010)['accepted']


def test_wide_gap_and_raw_time_mismatch_not_accepted():
    assert not sensitivity(*make(count=8),.010)['accepted']
    args=make();args[1].header.stamp.nanosec+=1000000
    assert sensitivity(*args,.010)['reason']=='raw_scene_mismatch'


def test_no_unregistered_limit():
    with pytest.raises(ValueError,match='registered'):
        sensitivity(*make(),.05)
