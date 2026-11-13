import math
from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon, box

from arena_autonomy.linear_observation import linear_observation, sector_rectangle, quiet_rotation, straight_path_supported
from arena_autonomy.lidar_motion import MotionHistory
from test_corridor_visibility import scene


def unknown_at(angle, value='unknown'):
    original = scene()
    rays = list(original.rays)
    index = round((angle+180)/.72)
    rays[index] = replace(rays[index],status=value,hit_xy_m=None)
    return replace(original,rays=tuple(rays))


def test_straight_finite_scene_supported_and_no_unknown_filled():
    result = linear_observation(scene(),.2)
    assert result['supported'] and result['return_cells_checked']>2
    assert not result['unknown_filled'] and not result['free_area_certified']


@pytest.mark.parametrize('side', [90,-90,110,-110])
def test_unknown_outside_new_front_strip_preserved(side):
    result = linear_observation(unknown_at(side),.2)
    assert result['supported'] and result['unknown_irrelevant']>0


@pytest.mark.parametrize('angle', [-45,0,45])
@pytest.mark.parametrize('status', ['unknown','masked'])
def test_unknown_intersecting_needed_strip_rejected(angle,status):
    result = linear_observation(unknown_at(angle,status),.2)
    assert not result['supported'] and result['reason']=='linear_region_unobserved'


@pytest.mark.parametrize('speed', [0.,.2,1.])
def test_short_returns_rejected(speed):
    assert not linear_observation(scene(.25),speed)['supported']


def test_measured_speed_not_replaced_by_lower_recovery_target():
    low, high = linear_observation(scene(),.2), linear_observation(scene(),1.)
    assert high['required_distance_m']>low['required_distance_m']


def test_future_and_stale_rejected():
    with pytest.raises(ValueError):
        replace(scene(),t_ref_s=.29)
    assert not linear_observation(replace(scene(),t_ref_s=.61),.2)['supported']


def test_incomplete_indices_rejected():
    original = scene()
    assert not linear_observation(replace(original,rays=original.rays[:200]+original.rays[201:]),.2)['supported']


@pytest.mark.parametrize('angle', np.linspace(-math.pi,math.pi,17))
def test_sector_clipping_matches_independent_polygon(angle):
    origin=(-.05,.02)
    bounds=(.1,-.2,.7,.2)
    direction=(math.cos(angle),math.sin(angle))
    half=.017
    clipped=sector_rectangle(origin,direction,half,bounds)
    triangle=Polygon([origin,*[(origin[0]+5*math.cos(angle+d),origin[1]+5*math.sin(angle+d)) for d in (-half,half)]])
    reference=triangle.intersection(box(*bounds))
    area=Polygon(clipped).area if len(clipped)>=3 else 0.
    assert area == pytest.approx(reference.area,abs=1e-12)


def test_quiet_gyro_requires_full_fresh_window():
    h=MotionHistory()
    assert not quiet_rotation(h,.4)
    for t in np.arange(0.,.401,.01): h.add_gyro(float(t),0.)
    assert quiet_rotation(h,.4)
    assert not quiet_rotation(h,.6)
    h.add_gyro(.41,.051)
    assert not quiet_rotation(h,.41)


def test_straight_path_must_be_current_close_and_long_enough():
    path=np.column_stack((np.arange(0.,.81,.02),np.zeros(41)))
    assert straight_path_supported(path,.3)
    assert not straight_path_supported(path+[0,.081],.3)
    assert not straight_path_supported(path[:10],.3)
