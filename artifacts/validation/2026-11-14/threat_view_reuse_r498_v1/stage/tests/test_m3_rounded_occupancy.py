"""기본 M3 출력 보존 및 원판 후보의 명시 선택. 출발 주행 시험은 아니다."""
from dataclasses import replace
import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.occupancy_timing import per_ray_padding
from probe_grid_follow import stationary_history
from test_startup_straight_state import scene,baseline,SENSOR


def controller(rounded):
    return mx.M3ExecutionFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate'),
        mx.ExecutionConfig(rounded_occupancy=rounded),allow_simulation_trial=True)


def test_default_false_and_typed_selection():
    assert not mx.ExecutionConfig().rounded_occupancy
    with pytest.raises(ValueError):mx.ExecutionConfig(rounded_occupancy=1)


def test_off_never_calls_new_occupancy_function(monkeypatch):
    import arena_autonomy.rounded_occupancy_candidate as module
    def unexpected(*a,**k):raise AssertionError('OFF called candidate')
    monkeypatch.setattr(module,'rounded_swept_limit',unexpected)
    c=controller(False); s=scene(); h=stationary_history(.135,SENSOR)
    path=np.array(baseline()[2]['path_points'])
    result=c._evaluate_command('straight',path,s,[],h,0.,0.,.02,per_ray_padding(s,.055,.5),baseline())
    assert result['accepted'] and 'occupancy_model' not in result['detail']


def test_on_uses_same_measurement_margin_and_both_angle_sweeps(monkeypatch):
    import arena_autonomy.rounded_occupancy_candidate as module
    original=module.rounded_swept_limit;seen=[]
    def spy(*a,**k):seen.append((a,k));return original(*a,**k)
    monkeypatch.setattr(module,'rounded_swept_limit',spy)
    c=controller(True);s=scene();h=stationary_history(.135,SENSOR)
    pad=per_ray_padding(s,.055,.5)
    result=c._evaluate_command('straight',np.array(baseline()[2]['path_points']),s,[],h,0.,0.,.02,pad,baseline())
    assert result['accepted'] and len(seen)==2
    for a,k in seen:
        np.testing.assert_array_equal(k['margin'],pad)
        assert k['length']==.2 and k['width']==.15 and a[2]==.135
    assert result['detail']['occupancy_model']=='fixed_rectangle_plus_euclidean_disk_candidate'


def test_unknown_front_hold_remains_for_both_modes():
    s=scene(fault=True);h=stationary_history(.135,SENSOR)
    for rounded in (False,True):
        result=controller(rounded).command(s,h,0.,'left')
        assert result[0]==0 and result[2]['reason']=='follow_front_unobserved'
