import sys
from pathlib import Path
import numpy as np
import pytest

A=Path(__file__).resolve().parent;sys.path.insert(0,str(A))
from polygon_candidate import polygon_distance
from arena_autonomy.recovery_geometry import polygon_distance as reference


@pytest.mark.parametrize('seed',[1,19,91])
@pytest.mark.parametrize('n',[0,1,2,3,4,7,16])
def test_distances_winding_degenerate_and_threshold(seed,n):
    rng=np.random.default_rng(seed);angles=np.sort(rng.uniform(-np.pi,np.pi,n))
    polygon=np.column_stack((np.cos(angles),np.sin(angles))) if n else np.empty((0,2))
    polygon=polygon@rng.normal(size=(2,2))
    points=np.vstack((rng.uniform(-3,3,(300,2)),polygon))
    for p in (polygon,polygon[::-1],polygon*1e-8,polygon*0.):
        a=reference(points,p);b=polygon_distance(points,p)
        np.testing.assert_allclose(b,a,rtol=1e-14,atol=1e-14)
        np.testing.assert_array_equal(b==0,a==0)
        for threshold in (1e-12,.025,.03,.05):np.testing.assert_array_equal(b<=threshold,a<=threshold)


def test_near_edge_area_and_nan_contract():
    for h in (0.,1e-16,5e-16,1e-15,1e-12,.15):
        polygon=np.array([[0.,0.],[1.,0.],[1.,h],[0.,h]])
        points=np.array([[.5,v] for v in (-1e-12,np.nextafter(-1e-12,0),0.,h,h+1e-12)]+[[np.nan,0.],[np.inf,0.]])
        with np.errstate(invalid='ignore'):
            a=reference(points,polygon);b=polygon_distance(points,polygon)
        np.testing.assert_allclose(b,a,rtol=1e-14,atol=1e-14,equal_nan=True)
        np.testing.assert_array_equal(b==0,a==0)
