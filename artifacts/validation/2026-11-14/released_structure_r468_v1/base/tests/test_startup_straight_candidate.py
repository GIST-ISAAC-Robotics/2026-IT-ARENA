from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.startup_straight_candidate import StartupStraightCandidate
from analyze_grid_start_constraints import stationary_scene
from probe_grid_follow import stationary_history, stationary_ranges

SENSOR=dict(xyz_m=[.06,0.,.095],rate_hz=10.,masked_rear_half_angle_deg=30.)
LIDAR=dict(range_min_m=.05,range_max_m=12.)
ROOM=np.array([[[-2.,-.425],[3.,-.425]],[[3.,-.425],[3.,.425]],[[3.,.425],[-2.,.425]],[[-2.,.425],[-2.,-.425]]])


def fixture():
    ranges,_=stationary_ranges(dict(x=0.,y=0.,yaw_rad=0.),[],ROOM,SENSOR,500,.05,12.)
    return stationary_scene(ranges,SENSOR,LIDAR,0),stationary_history(.135,SENSOR)


def test_default_off_cannot_replace_command_or_mutate_baseline():
    scene,history=fixture()
    baseline=(0.,0.,dict(reason='follow_gap_hold'))
    before=deepcopy(baseline)
    result=StartupStraightCandidate().evaluate(scene,history,baseline,0.)
    assert not result['selected'] and result['reason']=='disabled' and baseline==before


@pytest.mark.parametrize('reason',['follow_boundary_unconfirmed','follow_front_unobserved','follow_scene_stale',
                                  'local_follow','input_stop','STOP'])
def test_not_a_boundary_or_stop_bypass(reason):
    scene,history=fixture()
    result=StartupStraightCandidate(enabled=True).evaluate(scene,history,(0.,0.,dict(reason=reason)),0.)
    assert not result['selected'] and result['reason']=='baseline_not_gap_hold'


@pytest.mark.parametrize('speed,steering',[(.01,0.),(-.01,0.),(0.,.001),(float('nan'),0.)])
def test_nonstationary_or_turned_entry_is_rejected(speed,steering):
    scene,history=fixture()
    result=StartupStraightCandidate(enabled=True).evaluate(scene,history,(0.,0.,dict(reason='follow_gap_hold')),speed,steering)
    assert result['reason']=='stationary_straight_entry_required' and not result['selected']


def test_missing_path_and_stale_source_rejected():
    scene,history=fixture()
    observer=StartupStraightCandidate(enabled=True)
    baseline=(0.,0.,dict(reason='follow_gap_hold'))
    assert observer.evaluate(scene,history,baseline,0.)['reason']=='confirmed_path_required'
    assert observer.evaluate(replace(scene,t_ref_s=1.5),history,baseline,0.)['reason']=='source_invalid'


def test_existing_safety_parameters_are_preserved_and_speed_only_reduced():
    cfg=FollowConfig(occupancy_timing_mode='per_ray_candidate')
    observer=StartupStraightCandidate(cfg,enabled=True)
    assert observer.config==replace(cfg,max_speed_mps=.2)
    assert observer.checker.lateral_config.budget==StartupStraightCandidate(cfg).checker.lateral_config.budget
    with pytest.raises(ValueError):StartupStraightCandidate(enabled='yes')
