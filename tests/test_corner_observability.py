import math

import numpy as np
import pytest

from probe_corner_observability import annular_scan, circle_first_hit, visible_inner_diameter


def test_circle_intersections_outside_inside_miss_and_behind():
    result=circle_first_hit([0.,0.],[[1.,0.],[-1.,0.],[0.,1.]],[2.,0.],1.)
    assert result[0]==pytest.approx(1.) and math.isinf(result[1]) and math.isinf(result[2])
    np.testing.assert_allclose(circle_first_hit([0.,0.],[[1.,0.],[0.,-1.]],[0.,0.],2.),[2.,2.])


@pytest.mark.parametrize('radius',[.10,.15,.20,.30,.40,.60,.80])
def test_dense_first_returns_match_analytic_visible_extent_bound(radius):
    # 접선 근처 이산 표본은 연속 광선의 상한을 넘지 않는다.
    _,labels,directions,origin,center=annular_scan(radius,samples=20000)
    distances=circle_first_hit(origin,directions,center,radius)
    points=origin+directions[labels]*distances[labels,None]
    chord=np.linalg.norm(points[0]-points[-1])
    ideal=visible_inner_diameter(radius,float(np.linalg.norm(origin-center)))
    assert chord<=ideal+1e-9
    assert chord>=ideal*.96


def test_small_circle_cannot_supply_required_extent_even_with_continuous_rays():
    assert visible_inner_diameter(.30,.725)<.65
    assert visible_inner_diameter(.40,.825)>.65


@pytest.mark.parametrize('radius,distance',[(0.,1.),(-1.,2.),(1.,1.),(2.,1.)])
def test_invalid_geometry_rejected(radius,distance):
    with pytest.raises(ValueError):visible_inner_diameter(radius,distance)


def test_left_right_scan_geometry_is_mirrored():
    left=annular_scan(.3)
    right=annular_scan(.3,-1)
    # index 0=-pi는 그대로, 나머지는 -angle 대응이다.
    order=np.r_[0,np.arange(499,0,-1)]
    np.testing.assert_allclose(left[0],right[0][order],atol=1e-12)
    np.testing.assert_array_equal(left[1],right[1][order])
