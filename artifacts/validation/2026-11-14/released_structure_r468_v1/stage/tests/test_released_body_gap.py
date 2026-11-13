"""통과 뒤 좁은 후측방 결측의 기존 제한과 새 현재 입력 재검사."""
import copy
from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest
from arena_autonomy import pass_release_trial as ps
from arena_autonomy import released_body_gap as gap
from arena_autonomy.current_path_recheck import _released
from arena_autonomy.local_follow import FollowConfig
from test_rear_fringe_trial import case
from test_current_path_recheck import fixture_for


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True,
        guard_observation_policy='selected_r181',released_body_gap_trial=True,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')


@pytest.mark.parametrize('value',[True,1,'true'])
def test_option_requires_explicit_policy_and_strict_boolean(value):
    with pytest.raises(ValueError):FollowConfig(released_body_gap_trial=value)
    assert FollowConfig().released_body_gap_trial is False


@pytest.mark.parametrize('neighbor,accepted',[(4.,True),(.4,False),(.1,False)])
def test_real_gap_proof_still_rejects_near_returns(monkeypatch,neighbor,accepted):
    _,scan,scene=case(neighbor=neighbor)
    def region(*args,context,**kwargs):
        context.update(queries=np.array([[-.047,.0753]]),free=np.array([False]),
                       masked=np.array([False]),padding=np.full(500,.10))
        return dict(unobserved=1,masked_left=0,masked_right=0,queries=1)
    monkeypatch.setattr(ps,'motion_region',region)
    cfg=config();out=gap.check(scene,scan,.1,.15,0.,0.,.02,cfg,ps.lateral_config(cfg),ps.SelectorConfig())
    assert out['accepted'] is accepted and not out['unknown_filled']
    assert scene.rays[451].status=='unknown' and np.isposinf(scan.ranges[451])


@pytest.mark.parametrize('speed,target',[(.36,.35),(.1,.36),(-.1,.15),(.1,0.)])
def test_speed_bounds_do_not_assume_braking_finished(monkeypatch,speed,target):
    monkeypatch.setattr(ps,'motion_region',lambda *a,**k:pytest.fail('invalid motion entered geometry'))
    cfg=config()
    assert not gap.check(None,object(),speed,target,0.,0.,.02,cfg,None,None)['accepted']


def test_newly_observed_region_does_not_require_old_unknown(monkeypatch):
    monkeypatch.setattr(ps,'motion_region',lambda *a,**k:dict(unobserved=0,masked_left=0,masked_right=0))
    cfg=config();out=gap.check(None,object(),.1,.15,0.,0.,.02,cfg,None,ps.SelectorConfig())
    assert out['accepted'] and not out['proof']['accepted']


@pytest.mark.parametrize('fault',['off','mode','side','memory','old_source','front','speed'])
def test_proposal_requires_current_path_and_no_new_side(monkeypatch,fault):
    cfg=config();owner=ps.PassReleaseSelector(cfg)
    owner.sides['left'].update(seen=True,passed=True,present=False)
    for state in owner.sides.values():state['present']=False
    scene=NS(source_first_s=1.)
    baseline=[.7,.2,dict(reason='local_follow',boundary_from_memory=False,path_source_s=1.,
                        partial_observation=dict(allowed=True,reason='fully_observed'))]
    speed=.1
    if fault=='off':owner.follow_config=replace(cfg,released_body_gap_trial=False)
    elif fault=='mode':owner.sides['left']['passed']=False
    elif fault=='side':owner.sides['left']['present']=True
    elif fault=='memory':baseline[2]['boundary_from_memory']=True
    elif fault=='old_source':baseline[2]['path_source_s']=.9
    elif fault=='front':baseline[2]['partial_observation']['allowed']=False
    else:speed=.4
    monkeypatch.setattr(gap,'check',lambda *a,**k:pytest.fail('out-of-scope proposal checked'))
    assert gap.propose(owner,scene,object(),baseline,speed,0.,.02,.35) is None


def test_proposal_preserves_side_state_and_original_command_cap(monkeypatch):
    owner=ps.PassReleaseSelector(config())
    owner.sides['left'].update(seen=True,passed=True,present=False)
    before=copy.deepcopy(owner.sides);seen=[]
    def checked(scene,scan,speed,target,*args):
        seen.append((speed,target));return dict(accepted=True)
    monkeypatch.setattr(gap,'check',checked)
    baseline=(.7,.2,dict(reason='local_follow',boundary_from_memory=False,path_source_s=1.,
                        partial_observation=dict(allowed=True,reason='fully_observed')))
    out=gap.propose(owner,NS(source_first_s=1.),object(),baseline,.1,0.,.02,.25)
    assert out['target_mps']==.25 and out['steering_rad']==.2 and seen==[(.1,.25)]
    assert owner.mode=='RELEASED' and owner.sides==before


@pytest.mark.parametrize('current_proof',[True,False])
def test_rebase_checks_new_scene_instead_of_reusing_original_permission(monkeypatch,current_proof):
    args=list(fixture_for('released_baseline'));out=args[0]
    out[2]['normal']['selector_released_body_gap']=dict(accepted=True)
    args[-1]=replace(args[-1],guard_observation_policy='selected_r181',released_body_gap_trial=True)
    monkeypatch.setattr(_released.base,'rebase',lambda *a:dict(speed_mps=.15,steering_rad=.1,path_points=[[0,0],[1,0]]))
    monkeypatch.setattr(_released.coverage,'current_forward_coverage',lambda *a:True)
    monkeypatch.setattr(ps,'threat_view',lambda *a:dict(left=dict(present=False),right=dict(present=False)))
    seen=[]
    def checked(scene,*a):seen.append(scene);return dict(accepted=current_proof)
    monkeypatch.setattr(gap,'check',checked)
    result=_released.rebase(*args)
    assert seen==[args[3]] and (result['speed_mps']>0)==current_proof
    assert result['state_transition'] is False and result['independent_guard_required']
