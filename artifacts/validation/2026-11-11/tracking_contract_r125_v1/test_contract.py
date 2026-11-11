from dataclasses import replace
from types import SimpleNamespace
import math

import numpy as np
import pytest
from path_contract import PathProposal, current_candidate, proposal
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from test_local_follow import history


def inputs():
    cfg = FollowConfig()
    data = dict(reference_s=.5,epoch=0,side='left',timing_verified=True,
                speed_mps=.2,previous_steering_rad=0.)
    scene = scene_from_scan([3.]*500, angle_min=-math.pi,angle_increment=math.tau/500,
        range_min=.05,range_max=12.,acquired_at_s=(.4+np.arange(500)*.0002).tolist(),t_ref_s=.5)
    path = tuple((float(x),0.) for x in np.linspace(-.1,2.,43))
    plan = PathProposal(path,.4,.3,0,0,'left',.35,'baseline',.3)
    return plan,data,SimpleNamespace(source_epoch=0),history(v=.2),scene,cfg


def test_fresh_path_keeps_original_deadline():
    result = current_candidate(*inputs())
    assert result['eligible'] and result['path_source_s']==.3
    assert result['path_age_s']==pytest.approx(.2)
    assert result['path_deadline_s']==pytest.approx(.6)
    assert 0<result['speed_mps']<=.35 and not result['runtime_authorized']


@pytest.mark.parametrize('reference', [.4,.49])
def test_new_calculation_cannot_renew_old_path(reference):
    p,*other=inputs()
    p=replace(p,path_source_s=.1,reference_s=reference)
    assert current_candidate(p,*other)['reason']=='plan_path_source_expired'


def test_deadline_is_exclusive_and_configured_shorter_age_is_respected():
    p,d,scan,h,scene,cfg=inputs()
    assert not current_candidate(replace(p,path_source_s=.2),d,scan,h,scene,cfg)['eligible']
    assert current_candidate(p,d,scan,h,scene,replace(cfg,memory_ttl_s=.19))['reason']=='plan_path_source_expired'


@pytest.mark.parametrize('source', [None,True,math.nan,math.inf,-.1,.31])
def test_proposal_requires_original_time_not_reconstructed_latest_scan(source):
    d=dict(reference_s=.4,epoch=0,side='left',scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0,acquired_at_s=[.3])))
    normal=dict(reason='local_follow',selector_choice='baseline',path_source_s=source,
                path_points=[[i*.1,0.] for i in range(20)])
    assert proposal(d,[.2,0.,dict(handoff_state='FOLLOW',normal=normal)]) is None


def test_memory_path_source_survives_plan_extraction():
    d=dict(reference_s=.4,epoch=0,side='left',scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0,acquired_at_s=[.3])))
    n=dict(reason='parallel_hold_follow',selector_choice='parallel',path_source_s=.25,
           path_points=[[i*.1,0.] for i in range(20)])
    p=proposal(d,[.2,0.,dict(handoff_state='FOLLOW',normal=n)])
    assert p.path_source_s==.25 and p.reference_s==.4 and p.source_first_s==.3


def test_current_input_epoch_cannot_reuse_earlier_plan():
    p,d,scan,h,scene,cfg=inputs()
    assert current_candidate(p,{**d,'epoch':1},scan,h,scene,cfg)['reason']=='plan_epoch_or_side_changed'


def test_missing_motion_does_not_fall_back_to_unshifted_path():
    p,d,scan,h,scene,cfg=inputs();h.reset()
    assert current_candidate(p,d,scan,h,scene,cfg)['reason']=='current_geometry_unavailable'
