from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.follow_trace import FollowConfig
import arena_autonomy.local_follow as m
A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/corner_r45_transition_audit_v1'
def actual():
    q=next(r for r in json.loads((A/'path_comparison.json').read_bytes()) if abs(r['t']-36.72)<1e-8)
    p=next(c for c in q['candidates'] if c['kind']=='all' and c['side']=='right')
    ref=next(r for r in json.loads((A/'captures.json').read_bytes()) if abs(r['t']-36.72)<1e-8)['carried_path']
    with (A.parent/'corner_occlusion_connected_r45_v1/run1/follow_decisions.jsonl').open() as f:
        cfg=FollowConfig(**json.loads(next(f))['data']['follow_config'])
    return np.asarray(p['path']),np.asarray(ref),p['info'],cfg
def test_observed_same_corner_common_arc_avoids_long_exit_mismatch():
    p,ref,info,cfg=actual()
    assert not m.path_agrees(p,ref,cfg.boundary_agreement_m)
    assert m.retained_boundary_agreement(p,ref,info,cfg)=='shared_arc'
    assert info['boundary_observed_ahead_m']>cfg.boundary_min_ahead_m
def test_mirrored_corner_uses_identical_geometric_agreement():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p*[1,-1],ref*[1,-1],info,cfg)=='shared_arc'
@pytest.mark.parametrize('flag',['traffic_boundary_refresh_trial','piecewise_wall_trial'])
def test_option_off_retains_previous_rejection(flag):
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p,ref,info,replace(cfg,**{flag:False})) is None
def test_non_piecewise_model_does_not_receive_extra_agreement():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p,ref,{**info,'wall_model':'other'},cfg) is None
def test_different_corner_location_does_not_match():
    p,ref,info,cfg=actual()
    assert m.retained_boundary_agreement(p+[0,.3],ref,info,cfg) is None
def test_short_common_arc_is_not_enough():
    p,ref,info,cfg=actual();p=p[np.linalg.norm(p,axis=1)<.2]
    assert m.retained_boundary_agreement(p,ref,info,cfg) is None
def test_normal_straight_comparison_keeps_old_result():
    p=np.column_stack((np.linspace(-.2,1,36),np.zeros(36)))
    assert m.retained_boundary_agreement(p,p,{},FollowConfig())=='shared_x'
