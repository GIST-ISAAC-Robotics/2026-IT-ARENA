import math

import numpy as np
import pytest

from probe_grid_departure_observation import moving_ranges, motion_history, append_stationary_measurements
from g0_geometry import TruthBox,Footprint
from arena_autonomy.m3_execution_trial import presteer_motion_quiet

SENSOR=dict(xyz_m=[.06,0.,.095],rate_hz=10.,masked_rear_half_angle_deg=30.)
ROOM=np.array([[[-2,-1],[3,-1]],[[3,-1],[3,1]],[[3,1],[-2,1]],[[-2,1],[-2,-1]]])
EGO=dict(x=0.,y=0.,yaw_rad=0.)


def test_center_ray_uses_its_own_time_not_end_of_scan_pose():
    stamps=.5+np.arange(500)*.0002
    q,_=moving_ranges(EGO,[TruthBox(.7,0.,.12)],[(.5,0.)],ROOM,SENSOR,stamps,.05,12.)
    assert q[250]==pytest.approx(.7+.5*.05-.1-.06)
    assert q[250]!=pytest.approx(.7+.5*.0998-.1-.06)


def test_zero_velocity_matches_exact_front_surface_and_range_bias():
    stamps=.5+np.arange(500)*.0002
    q,_=moving_ranges(EGO,[TruthBox(.7,0.,.12)],[(0.,0.)],ROOM,SENSOR,stamps,.05,12.,.03)
    assert q[250]==pytest.approx(.57)


def test_bias_does_not_reveal_wall_behind_subminimum_obstacle():
    stamps=.5+np.arange(500)*.0002
    q,label=moving_ranges(EGO,[TruthBox(.1,0.,.12,Footprint(.04,.15))],[(0.,0.)],ROOM,SENSOR,stamps,.05,12.,.03)
    assert math.isnan(q[250]) and label[250]=='target'


def test_unordered_time_and_mismatched_velocity_rejected():
    stamps=.5+np.arange(500)*.0002
    with pytest.raises(ValueError):moving_ranges(EGO,[],[(0.,0.)],ROOM,SENSOR,stamps,.05,12.)
    with pytest.raises(ValueError):moving_ranges(EGO,[],[],ROOM,SENSOR,stamps[::-1],.05,12.)


def test_rolling_motion_inputs_do_not_prune_early_control_times_or_use_future_measurements():
    h=motion_history(.135,SENSOR)
    for now in (.6018,.6218,.7018,1.0018,2.0018,3.5018):
        append_stationary_measurements(h,now)
        assert presteer_motion_quiet(h,now)
        assert h.wheels.times[-1]<=now and h.gyro.times[-1]<=now
        h.poses([now-.1,now],now)
    assert h.config.history_s==2. and h.wheels.times[0]>=1.49
