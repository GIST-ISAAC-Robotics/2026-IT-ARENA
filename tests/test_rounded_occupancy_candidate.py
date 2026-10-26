import math

import numpy as np
import pytest

from arena_autonomy.local_path import swept_limit
from arena_autonomy.rounded_occupancy_candidate import point_rectangle_distance, rounded_swept_limit

GEOMETRY=dict(wheelbase=.135,length=.2,width=.15,body_padding=.025)


def test_rectangle_distance_at_face_inside_and_diagonal():
    q=np.array([[0,0],[.125,.1],[.225,.1],[.225,.2]])
    d=point_rectangle_distance(q,0.,0.,**GEOMETRY)
    np.testing.assert_allclose(d,[0,0,.1,math.sqrt(.02)],atol=1e-15)


def test_square_corner_false_overlap_is_separate_from_original_three_cm_gap():
    # 저장 장면에서 발견한 뒤 대각선 반환. 총 여유는 같은 .1006 m다.
    q=[[-.20005,.17435]]
    assert swept_limit(q,0.,.135,margin=.1006)==0.
    assert rounded_swept_limit(q,0.,.135,margin=.1006)==3.
    # 최초 그리드의 실제 가까운 옆 모서리는 두 방식 모두 거절해야 한다.
    close=[[-.12,-.105]]
    assert swept_limit(close,0.,.135,margin=.1017)==0.
    assert rounded_swept_limit(close,0.,.135,margin=.1017)==0.


@pytest.mark.parametrize('angle',[0.,-.06,.06,-.37,.37])
def test_certified_prefix_has_no_disk_overlap_at_dense_intermediate_positions(angle):
    rng=np.random.default_rng(239)
    points=rng.uniform([.28,-.8],[1.8,.8],size=(150,2))
    margin=rng.uniform(.055,.15,len(points))
    limit=rounded_swept_limit(points,angle,.135,horizon=1.2,margin=margin)
    for d in np.linspace(0,limit,201):
        assert np.all(point_rectangle_distance(points,angle,float(d),**GEOMETRY)>margin-.025-1e-12)


def test_between_sample_collision_is_not_missed():
    q=[[.145,0.]]
    # 전면 .125에서 2 cm 전진 시 접촉. 기존 격자 노드 사이를 검사한다.
    assert rounded_swept_limit(q,0.,.135,horizon=.04,margin=.025)==0.


@pytest.mark.parametrize('angle',[0.,.2,-.2])
def test_more_uncertainty_cannot_increase_certified_clearance(angle):
    p=np.array([[.8,.1],[.5,.4],[1.1,-.3]])
    distances=[rounded_swept_limit(p,angle,.135,margin=m) for m in (.025,.055,.1,.15)]
    assert distances==sorted(distances,reverse=True)


def test_xy_component_contract_is_not_euclidean_speed_contract():
    bound=.5; age=.1
    assert np.linalg.norm([bound*age,bound*age]) > bound*age


@pytest.mark.parametrize('points,margin',[([[math.nan,0]],.1),([[.3,0]],.02),([[.3,0]],[-.1]),([[.3,0]],[.1,.2])])
def test_bad_inputs_are_rejected(points,margin):
    with pytest.raises(ValueError):rounded_swept_limit(points,0.,.135,margin=margin)


def test_distance_agrees_with_independent_polygon_library():
    from shapely.geometry import Point,Polygon
    from shapely.affinity import rotate,translate
    rng=np.random.default_rng(417)
    rectangle=Polygon([[-.125,-.1],[.125,-.1],[.125,.1],[-.125,.1]])
    for angle in (-.37,-.06,0.,.06,.37):
        curve=math.tan(angle)/.135
        for distance in (0.,.031,.177,.56):
            yaw=curve*distance
            rear_x=distance-.0675 if curve==0 else math.sin(yaw)/curve-.0675
            rear_y=0. if curve==0 else (1-math.cos(yaw))/curve
            cx,cy=rear_x+.0675*math.cos(yaw),rear_y+.0675*math.sin(yaw)
            shape=translate(rotate(rectangle,yaw,origin=(0,0),use_radians=True),cx,cy)
            pts=rng.uniform([-.5,-.5],[1.,.7],(60,2))
            actual=point_rectangle_distance(pts,angle,distance,**GEOMETRY)
            np.testing.assert_allclose(actual,[shape.distance(Point(p)) for p in pts],atol=1e-12,rtol=0.)


def test_straight_prefix_does_not_cross_analytical_first_contact():
    for x in (.3,.7,1.1):
        for y in (0.,.1,.14,.17):
            radius=.08
            gap_y=max(0.,abs(y)-.1)
            contact=x-.125-math.sqrt(radius*radius-gap_y*gap_y)
            limit=rounded_swept_limit([[x,y]],0.,.135,horizon=1.5,margin=radius+.025)
            assert 0<=limit<=contact+1e-12
