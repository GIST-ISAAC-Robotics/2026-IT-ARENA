"""영구 기둥 가림 적응과 예기치 않은 결측/장애물 보호를 구분한다."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
import yaml

from arena_autonomy.known_structure import classify, structure_policy, validate
from arena_autonomy.local_path import mask_scan
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.local_follow import FollowConfig
from test_local_follow import history, scan
from test_partial_observation import candidate_node
from test_lidar_mount_trial import launch_module, MODEL, MARKER

C = [[51.1155, 3.01854], [-51.1155, 3.01854], [150.5928, 2.08836], [-150.5928, 2.08836], [180., 11.3448]]

def policy(sectors=C):
    return structure_policy(PartialObservationPolicy, json.dumps(sectors))(
        allow_narrow_forward=True, surface_translation_enabled=True, lateral_metric_enabled=True)

def masked(first=.5, sectors=C):
    return mask_scan(scan(first, [8.]*500), {'rear_half_angle_deg':0., 'sectors_deg':sectors})

def test_structure_stays_unknown_but_is_not_a_generic_front_failure():
    raw=masked(); original=np.asarray(raw.ranges).copy()
    result=policy().evaluate(raw, [], history(), .6, 2.4)
    assert result['allowed'] and result['speed_cap_mps'] is None
    assert result['known_structure']['explained_sectors']==5
    assert result['known_structure']['unknown_filled'] is False
    np.testing.assert_array_equal(raw.ranges, original)
    assert np.isnan(raw.ranges).any()

@pytest.mark.parametrize('kind',['outside','flank','all'])
def test_unexplained_loss_is_not_excused_by_the_post_profile(kind):
    raw=masked()
    if kind=='outside': raw.ranges[250]=float('nan')
    elif kind=='flank':
        _,_,rows=classify(raw,C); raw.ranges[rows[0]['flanks'][0]]=float('nan')
    else: raw.ranges=[float('nan')]*500
    assert not policy().evaluate(raw, [], history(), .6, 2.4)['allowed']

def test_under_range_and_metadata_failure_still_stop():
    raw=masked(); raw.ranges[250]=.01
    assert not policy().evaluate(raw, [], history(), .6, 2.)['allowed']
    raw=masked();raw.angle_increment=math.pi/500
    assert not policy().evaluate(raw, [], history(), .6, 2.)['allowed']

def test_turning_into_far_shadow_is_speed_limited():
    result=policy().evaluate(masked(), [], history(), .6, 2.4, .37, 0.)
    assert result['allowed']
    assert 0 <= result['known_structure']['shadow_cap_mps'] < 2.4

def test_hidden_previously_observed_obstacle_cannot_expire_into_permission():
    p=policy([[0.,3.]])
    p.structure_previous=(np.array([[.2,0.]]), .5)
    r=p.evaluate(masked(.5,[[0.,3.]]),[],history(),.6,1.)
    assert r['known_structure']['hidden_memory_points']==1
    assert r['known_structure']['memory_cap_mps'] < .3
    r=p.evaluate(masked(.7,[[0.,3.]]),[],history(),.8,1.)
    assert r['known_structure']['hidden_memory_points']==1
    assert r['known_structure']['memory_cap_mps'] < .3

def test_visible_near_obstacle_still_brakes_in_the_independent_layer():
    node,commands,statuses=candidate_node(forward=True)
    node.partial_policy=policy();node.scan=masked();node.command.drive.speed=2.
    node.motion.project=lambda raw:(np.tile([[.14,0.]],(60,1)),60)
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.
    assert json.loads(statuses[-1].data)['reason']=='lidar_braking'

def test_policy_requires_explicit_profile_and_v4():
    assert FollowConfig().known_structure_policy=='off'
    assert validate('off','')==()
    with pytest.raises(ValueError): validate('off',json.dumps(C))
    with pytest.raises(ValueError): validate('structure_aware_v1','[]')
    with pytest.raises(ValueError): FollowConfig(known_structure_policy='structure_aware_v1',known_structure_sectors_json=json.dumps(C))
    assert policy().evaluate(masked(),[],history(),.6,1e100)['reason']=='structure_invalid_speed'

def test_launch_structural_mask_geometry_matches_unknown_option():
    launch=launch_module()
    root=Path(__file__).resolve().parents[1]
    profile=yaml.safe_load((root/'artifacts/validation/2026-11-04/lidar_mount_performance_v1/profiles/c_unknown.yaml').read_text(encoding='utf8'))
    _,before=launch._mount_trial(MODEL,profile,.0325,.06,*MARKER)
    profile['lidar']['known_structure_mask']='structure_aware'
    model,after=launch._mount_trial(MODEL,profile,.0325,.06,*MARKER)
    assert after==before and model.count('c1_mount_post_')==4

def test_default_partial_observation_still_rejects_c_gap():
    raw=masked(); p=PartialObservationPolicy(allow_narrow_forward=True,surface_translation_enabled=True,lateral_metric_enabled=True)
    assert not p.evaluate(raw,[],history(),.6,2.4)['allowed']
