"""새 제어 연결이 아닌 교집합 기하 비교. 외부 기하 연산으로 교차 검사한다."""
import math
import numpy as np
import pytest
from shapely.geometry import Point, Polygon
from probe_joint_missing_sweep import polygon_distance, joint_missing_sweep, sweep_rectangles
from arena_autonomy.partial_observation import near_sweep


def geometry(points, angle=math.pi/2, speed=.15, steering=.175):
    return joint_missing_sweep(points, [[.031, -.005]], [angle], math.radians(.36), .6,
                               speed, .178, steering, steering)


def test_separate_overlaps_do_not_prove_a_shared_location():
    old = np.array([[.386, .432]])
    radius = .03 + 2 * .16
    assert near_sweep(old, .15, .178, .175, .175, .135, .2, .15, radius)[0]
    assert abs(old[0, 0] - .031) < radius + .01  # 기존 광선 확장과도 겹친다.
    distance, _ = geometry(old, angle=1.50039)
    assert distance[0] > radius + .10


@pytest.mark.parametrize('point', [[.06,.15],[.06,.2],[.06,.08]])
def test_known_object_that_can_reach_missing_sweep_is_retained(point):
    distances, _ = geometry([point])
    assert distances[0] <= .03 + 2*.16


def test_forward_missing_object_is_retained_and_far_side_is_not():
    distances, _ = geometry([[.4,-.005],[.1,.8]], angle=0., speed=1., steering=0.)
    assert distances[0] == pytest.approx(0)
    assert distances[1] > .7


def test_longer_delay_increases_old_point_reach_without_fabricating_a_return():
    points = np.array([[.386,.432]])
    before = points.copy()
    distance, _ = geometry(points, angle=1.50039)
    assert distance[0] > .03 + 2*.16
    assert distance[0] < .03 + 2*.29
    np.testing.assert_array_equal(points,before)


def test_same_object_assumption_is_not_used_for_two_objects():
    distances,_ = geometry([[.386,.432],[.06,.15]])
    assert distances[0] > .35 and distances[1] < .35


@pytest.mark.parametrize('steering',[-.37,0.,.37])
def test_convex_clipping_and_distance_match_independent_geometry(steering):
    rng=np.random.default_rng(89)
    points=rng.uniform([-.3,-.8],[1.2,.8],size=(120,2))
    origins=np.array([[.03,-.005],[.06,0.]])
    directions=np.array([1.50039,-.5])
    distances,_=joint_missing_sweep(points,origins,directions,math.radians(.36),.8,
                                    .8,.19,steering,-steering)
    polygons=[]
    for center,axes,sizes in sweep_rectangles(.8,.19,steering,-steering,.135,.2,.15):
        box=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*sizes
        box=Polygon(box@axes+center)
        for origin,angle in zip(origins,directions):
            a=np.array([math.cos(angle),math.sin(angle)]);n=np.array([-a[1],a[0]])
            w=.8*math.tan(math.radians(.36))
            wedge=Polygon([origin,origin+.8*a-w*n,origin+.8*a+w*n])
            intersection=wedge.intersection(box)
            if not intersection.is_empty:polygons.append(intersection)
    # 합집합까지 거리는 각 교집합까지 거리의 최솟값이다. 얇은 중첩 면의
    # 불필요한 GEOS union/재분할 대신 독립 교집합 연산을 그대로 비교한다.
    expected=np.array([min(shape.distance(Point(p)) for shape in polygons) for p in points])
    np.testing.assert_allclose(distances,expected,rtol=0,atol=1e-10)


def test_polygon_distance_handles_segment_point_and_both_orientations():
    points=np.array([[0.,0.],[.5,.5],[2.,0.]])
    square=np.array([[0.,0.],[1.,0.],[1.,1.],[0.,1.]])
    for p in (square,square[::-1]):
        np.testing.assert_allclose(polygon_distance(points,p),[0,0,1])
    np.testing.assert_allclose(polygon_distance(points,np.array([[0.,0.],[1.,0.]])),[0,.5,1])
    np.testing.assert_allclose(polygon_distance(points,np.array([[0.,0.]])),[0,math.sqrt(.5),2])


@pytest.mark.parametrize('fault',['nan','huge_speed','negative_age','wide_cell','shape','steering'])
def test_invalid_inputs_do_not_return_permission(fault):
    args=dict(points=[[.4,.4]],origins=[[.06,0]],directions=[1.5],half_angle=.006,
              required=.6,speed=.5,age=.1,steering=.1,previous_steering=0.)
    if fault=='nan':args['points']=[[math.nan,0.]]
    if fault=='huge_speed':args['speed']=100.
    if fault=='negative_age':args['age']=-.1
    if fault=='wide_cell':args['half_angle']=.2
    if fault=='shape':args['directions']=[]
    if fault=='steering':args['steering']=1.5
    with pytest.raises(ValueError):joint_missing_sweep(**args)
