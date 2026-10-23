import math
import numpy as np
import pytest
from compare_corridor_stadium import Stadium,stadium_hits


def test_stadium_straight_and_cap_first_hits():
    origins=np.array([[0.,0.],[0,0],[2.,0]])
    directions=np.array([[1.,0],[0,1],[-1.,0]])
    np.testing.assert_allclose(stadium_hits(origins,directions,0.,.5),[1.5,.5,.5])


def test_circle_inner_half_is_not_a_false_cap():
    # 원 왼쪽 교점 x=.5는 내부 가상 원이다. 실제 오른쪽 바깥 반원 x=1.5만 사용.
    hit=stadium_hits(np.array([[0.,0.]]),np.array([[1.,0.]]),0.,.5)
    assert hit[0]==pytest.approx(1.5)


@pytest.mark.parametrize('sign',[1,-1])
def test_stadium_road_and_symmetric_arc_coordinate(sign):
    world=Stadium(sign)
    assert world.audit(world.initial_pose())['inside_road']
    r=.575
    assert world.coordinate([1+r,sign*r,sign*math.pi/2])==pytest.approx(2+r*math.pi/2)
    assert world.coordinate([-1-r,sign*r,sign*math.pi*1.5])==pytest.approx(4+r*math.pi*1.5)
    assert not world.audit([0.,sign*.3,0.])['inside_road']


def test_course_coordinate_unwraps_at_start():
    world=Stadium(1)
    truth=np.array([[0,-1.01,0,0],[1,-.99,0,0]])
    progress=world.progress(truth)
    assert 0<progress<.03
