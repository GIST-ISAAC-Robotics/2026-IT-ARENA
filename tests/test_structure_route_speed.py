"""C에서도 후보 속도를 높인 뒤 최종 경로/독립 보호를 통과하는지 검증."""
import json
import math
import numpy as np
import pytest
from dataclasses import replace
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_path import mask_scan
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history
from test_boundary_bridge import CONFIG

def sectors(s):
    return [(math.degrees(math.atan2(y,x-s)),math.degrees(math.asin(.0035/math.hypot(x-s,y)))+.5)
            for x,y in [(x,y) for x in [.080,-.080] for y in [.062,-.062]]]

def trial(s=.05,policy=True):
    cfg=FollowConfig(**CONFIG, boundary_bridge=True, partial_observation_policy='bounded_local_v4',
                     known_structure_policy='structure_aware_v1' if policy else 'off',
                     known_structure_sectors_json=json.dumps(sectors(s)) if policy else '')
    ctl=LocalFollow(cfg); h=history()
    h.config=replace(h.config,lidar_x_m=s)
    return ctl,h

def corridor(first,s=.05,fault=None):
    angles=-math.pi+np.arange(500)*math.tau/500
    raw=scan(first,np.minimum(8./np.maximum(abs(np.cos(angles)),1e-8),
                              .425/np.maximum(abs(np.sin(angles)),1e-8)).tolist())
    raw=mask_scan(raw,{'rear_half_angle_deg':0.,'sectors_deg':sectors(s)})
    if fault=='missing':raw.ranges[250]=float('nan')
    if fault=='near':
        for i in range(242,259): raw.ranges[i]=.16
    return raw

def command(ctl,h,first,s=.05,fault=None):
    raw=corridor(first,s,fault); before=np.asarray(raw.ranges).copy()
    local=motion_scene(raw,h,first+.1,timing_verified=True,rear_half_angle_deg=0.)
    out=ctl.command(local,h,1.,'left',raw_scan=raw)
    np.testing.assert_array_equal(before,raw.ranges)
    return out,local

@pytest.mark.parametrize('s',[.03,.05,.065])
def test_full_controller_can_exceed_base_speed_with_c_pillars(s):
    ctl,h=trial(s)
    for first in [.3,.4,.5,.6,.7,.8]:out,local=command(ctl,h,first,s)
    v,_,meta=out
    assert v>1.2,meta
    assert meta['open_route_speed_enabled'] and meta['open_route_requires_structure_check']
    assert meta['partial_observation']['known_structure']['evaluated']
    assert meta['partial_observation']['known_structure']['unknown_filled'] is False
    assert any(r.status!='return' and r.hit_xy_m is None for r in local.rays)

@pytest.mark.parametrize('fault',['missing','near'])
def test_new_cruise_candidate_is_not_final_permission(fault):
    ctl,h=trial()
    for first in [.3,.4,.5,.6,.7]: command(ctl,h,first)
    out,_=command(ctl,h,.8,fault=fault)
    assert out[0]==0.,out[2]

def test_no_structure_policy_preserves_the_original_front_contract():
    ctl,h=trial(policy=False)
    for first in [.3,.4,.5,.6,.7]:out,_=command(ctl,h,first)
    assert out[0]==0.
