"""4 cm 순차 기준과 묶음 충돌 검사의 수치·최초 간섭 동등성."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.local_path import swept_limit

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sweep_reference', ROOT/'artifacts/validation/2026-11-01/sweep_batch_v1/reference.py')
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


@pytest.mark.parametrize('count', [0, 1, 37, 417, 720])
def test_clouds_equal_scalar_and_per_point_padding(count):
    rng = np.random.default_rng(634+count)
    for index in range(80):
        points = rng.uniform([-.5,-2.5], [4.,2.5], (count,2))
        # 열린 직선도 포함하여 첫 덩어리보다 뒤쪽의 첫 간섭/무간섭을 확인한다.
        if index % 4 == 0:
            points[:, 1] += np.where(points[:, 1] >= 0, .4, -.4)
        margin = .025 if index % 2 else rng.uniform(.025,.15,count)
        args = dict(steering=rng.uniform(-.6,.6), wheelbase=rng.uniform(.1,.2),
                    horizon=rng.uniform(.01,5.), margin=margin, length=.20, width=.15)
        assert swept_limit(points, **args) == reference.swept_limit(points, **args), (count,index,args)


@pytest.mark.parametrize('curvature', [0., 1e-5-1e-12, 1e-5, 1e-5+1e-12, -1e-5, 1., -2.5])
def test_box_edges_and_chunk_edges_equal(curvature):
    wheelbase = .135; margin = .055; steering = math.atan(curvature*wheelbase)
    actual = math.tan(steering)/wheelbase
    for distance in [.0,.04,.28,.32,.60,.64,1.24,2.48]:
        yaw = actual*distance; c,s=math.cos(yaw),math.sin(yaw)
        if abs(actual)<1e-5:rx,ry=distance-wheelbase/2,0.
        else:rx,ry=s/actual-wheelbase/2,(1-c)/actual
        for x,y in [(.1+margin,.075+margin),(-.1-margin,.075+margin),(.1+margin,-.075-margin)]:
            for direction in [-math.inf,0.,math.inf]:
                xx = x if direction==0 else np.nextafter(x,direction)
                points=np.array([[rx+c*(xx+wheelbase/2)-s*y,ry+s*(xx+wheelbase/2)+c*y]])
                args=dict(steering=steering,wheelbase=wheelbase,horizon=2.51,margin=margin)
                assert swept_limit(points,**args)==reference.swept_limit(points,**args), (curvature,distance,xx,y)


@pytest.mark.parametrize('horizon', [-.1, 0., .001, .039, .04, .319, .32, 3.001])
def test_horizon_and_no_obstacle_contract(horizon):
    points=np.array([[100.,100.]])
    assert swept_limit(points,.2,horizon=horizon)==reference.swept_limit(points,.2,horizon=horizon)


def test_empty_points_and_nonfinite_points_keep_existing_behavior():
    for points in [np.empty((0,2)),np.array([[math.nan,math.inf]]),np.array([[.05,0.],[math.nan,0.]])]:
        assert swept_limit(points,0.)==reference.swept_limit(points,0.)
