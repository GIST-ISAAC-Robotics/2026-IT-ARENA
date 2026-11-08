from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.follow_trace import FollowConfig
from loader import load

A=Path(__file__).resolve().parent
m=load()
def actual():
    d=json.loads((A/'fixture.json').read_bytes())
    return np.asarray(d['current']['path']),np.asarray(d['reference']),d['current']['info'],FollowConfig(**d['config'])

def test_current_opposite_to_retained_wall_has_valid_common_corner():
    p,ref,info,cfg=actual()
    assert info['boundary_observed_ahead_m']>=cfg.boundary_min_ahead_m
    assert m.anchor_error(info,cfg)<=cfg.boundary_anchor_error_m
    assert not m.path_agrees(p,ref,cfg.boundary_agreement_m)
    assert m.retained_boundary_agreement(p,ref,info,cfg)=='shared_arc'

def test_mirror_retains_the_same_observed_agreement():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p*[1,-1],ref*[1,-1],info,cfg)=='shared_arc'

@pytest.mark.parametrize('flag',['traffic_boundary_refresh_trial','piecewise_wall_trial'])
def test_off_leaves_existing_current_corner_rejection(flag):
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p,ref,info,replace(cfg,**{flag:False})) is None

def test_foreign_surface_does_not_match_retained_corner():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p+[0,.3],ref,info,cfg) is None

def test_other_model_does_not_gain_arc_acceptance():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p,ref,{**info,'wall_model':'other'},cfg) is None

def test_short_common_observation_is_not_replaced_with_memory():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p[:4],ref,info,cfg) is None
