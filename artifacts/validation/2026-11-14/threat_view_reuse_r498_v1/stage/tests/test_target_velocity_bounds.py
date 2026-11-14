from types import SimpleNamespace
import math

import numpy as np
import pytest
from probe_observation_target_extent import finite_return_clearance, ego_clearance
from probe_target_velocity_bounds import DiskClearanceModel, velocity_bounds
from test_observation_target_extent import beam


MOTION = SimpleNamespace(lidar_x_m=0.,lidar_y_m=0.,lidar_yaw_rad=0.)
CFG = SimpleNamespace(length_m=.2,width_m=.15,measurement_padding_m=.03)
META = dict(count=1,angle_min=0.,angle_increment=.012,range_min=.05)


def test_matches_independent_scalar_ray_and_ego_distance():
    scene = beam()
    model = DiskClearanceModel(scene,(1.,.1),MOTION,CFG,META)
    for v in ((0.,0.),(.3,.2),(-.5,0.),(0.,-.5)):
        rays = finite_return_clearance(scene,(1.,.1),v)
        expected = min(rays[0]['clearance_m'],ego_clearance(scene,(1.,.1),v,MOTION,CFG,META))
        assert model.distances(v).min() == pytest.approx(expected)


def test_known_maximum_against_single_old_horizontal_ray():
    # t=.5의 ray, now=.6. y=.1에서 아래로 .5m/s면 과거 y=.15가 최댓값.
    result = velocity_bounds(DiskClearanceModel(beam(),(1.,.1),MOTION,CFG,META),.5)
    assert result['radius_lower_m'] <= .15+1e-12 <= result['radius_upper_m']+1e-12
    assert result['tolerance_met']
    assert .15-result['radius_lower_m'] <= 1e-4


def test_no_return_does_not_add_free_segment():
    model = DiskClearanceModel(beam(status='unknown'),(1.,.1),MOTION,CFG,META)
    assert len(model.distances((0.,0.))) == 2  # 과거/현재 몸체만


def test_current_body_overlap_bounds_all_velocities():
    result = velocity_bounds(DiskClearanceModel(beam(),(.09,.05),MOTION,CFG,META),.5)
    assert result['radius_lower_m'] == 0
    assert result['radius_upper_m'] <= 1e-12


def test_zero_speed_domain_and_budget_stop():
    model = DiskClearanceModel(beam(),(1.,.1),MOTION,CFG,META)
    zero = velocity_bounds(model,0.)
    assert zero['radius_lower_m'] == pytest.approx(.1)
    assert zero['tolerance_met']
    early = velocity_bounds(model,.5,max_cells=1)
    assert not early['tolerance_met']
    assert early['radius_upper_m'] >= .15


@pytest.mark.parametrize('seed',range(8))
def test_bounds_cover_independent_dense_velocity_samples(seed):
    rng = np.random.default_rng(seed)
    q = rng.uniform([.3,-.4],[1.5,.4])
    model = DiskClearanceModel(beam(),q,MOTION,CFG,META)
    result = velocity_bounds(model,.5,max_cells=100)
    angles = rng.uniform(-math.pi,math.pi,300)
    radii = .5*np.sqrt(rng.uniform(size=300))
    samples = np.column_stack((radii*np.cos(angles),radii*np.sin(angles)))
    assert max(model.distances(v).min() for v in samples) <= result['radius_upper_m']+1e-12
    assert model.distances(result['velocity_xy_mps']).min() == pytest.approx(result['radius_lower_m'])
    assert np.linalg.norm(result['velocity_xy_mps']) <= .5


@pytest.mark.parametrize('speed,tolerance,cells',[(math.nan,1e-4,10),(-1.,1e-4,10),(.5,0.,10),(.5,1e-4,0),(.5,1e-4,True)])
def test_invalid_bounds_rejected(speed,tolerance,cells):
    with pytest.raises(ValueError):
        velocity_bounds(DiskClearanceModel(beam(),(1.,.1),MOTION,CFG,META),speed,tolerance,cells)
