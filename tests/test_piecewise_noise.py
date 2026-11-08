import numpy as np
import pytest
from arena_autonomy.piecewise_wall import corner_path, _vertices
from test_piecewise_wall import polyline, segment_distance


def noisy(cloud, seed=23):
    radii=np.linalg.norm(cloud,axis=1)
    return cloud*(1+np.random.default_rng(seed).uniform(-.03,.03,len(cloud))/radii)[:,None]


@pytest.mark.parametrize('side,mirror',[('right',1),('left',-1)])
@pytest.mark.parametrize('seed',[0,1,23])
def test_noisy_single_corner_retains_original_points_and_fits_original_budget(side,mirror,seed):
    clean=polyline([-.4,-.505],[1.1,-.505],[1.1,1.3])*[1,mirror]
    cloud=noisy(clean,seed); saved=cloud.copy()
    assert len(_vertices(cloud))>3
    path=corner_path(cloud,side,.505,.347)
    assert path is not None
    expected=corner_path(clean,side,.505,.347)
    assert np.max(segment_distance(expected,path))<.03
    np.testing.assert_array_equal(cloud,saved)


@pytest.mark.parametrize('points',[
    [[-.4,-.505],[2.,-.505]],
    [[-.4,-.505],[.8,-.505],[.8,-.42]],
    [[-.4,-.505],[.8,-.505],[.8,.5],[1.5,.5]],
])
def test_noise_does_not_create_corner_from_straight_short_or_multiple_faces(points):
    assert corner_path(noisy(polyline(*points)),'right',.505,.347) is None


def test_observed_non_line_point_is_not_deleted_to_make_fit_pass():
    cloud=noisy(polyline([-.4,-.505],[1.1,-.505],[1.1,1.3]))
    cloud[35,1]+=.15
    assert corner_path(cloud,'right',.505,.347) is None
