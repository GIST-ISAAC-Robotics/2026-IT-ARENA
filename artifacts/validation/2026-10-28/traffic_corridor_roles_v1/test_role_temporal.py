"""역할 후보를 평가 프로세스에만 연결한 시간/분기/점유 반례."""
from dataclasses import replace
import numpy as np
import pytest

from arena_autonomy import corridor_observer as module
from arena_autonomy.corridor_roles import role_aware_evidence
from arena_autonomy.local_follow import FollowConfig
from test_corner_corridor_shadow import ring_scene
from test_corner_raycast import elbow
from test_local_follow import branch, cast_scene, history, post


@pytest.fixture(autouse=True)
def roles(monkeypatch):
    monkeypatch.setattr(module, 'corridor_evidence', role_aware_evidence)


def config():
    return FollowConfig(boundary_evidence_mode='corner_distance_candidate',
                        occupancy_timing_mode='per_ray_candidate',corridor_fallback_enabled=True)


@pytest.mark.parametrize('sign,side',[(1,'right'),(-1,'left')])
def test_outer_preferred_needs_distinct_geometry_and_motion_release(sign,side):
    observer=module.CorridorObserver(config())
    outputs=[observer.evaluate(ring_scene(.15,sign,now),history(),0.,side,sign*.2,.02)
             for now in (.4,.5,.6)]
    assert not outputs[0]['positive'] and not outputs[1]['positive']
    assert outputs[2]['positive']
    assert not outputs[2]['memory_refresh_authorized']
    assert observer.key[2]==side


@pytest.mark.parametrize('sign,side',[(1,'right'),(-1,'left')])
def test_reused_scans_do_not_accumulate_fresh_confirmations(sign,side):
    observer=module.CorridorObserver(config())
    scene=ring_scene(.15,sign,.4)
    for now in (.4,.42,.44,.46):
        out=observer.evaluate(replace(scene,t_ref_s=now),history(),0.,side,sign*.2,.02)
        assert not out['positive'] and observer.confirmations<=1


@pytest.mark.parametrize('fault',['epoch','side','reset','stale','reverse'])
def test_role_candidate_does_not_carry_permission_across_fault(fault):
    observer=module.CorridorObserver(config())
    for now in (.4,.5,.6):
        out=observer.evaluate(ring_scene(.15,1,now),history(),0.,'right',.2,.02)
    assert out['positive']
    scene,side,speed=ring_scene(.15,1,.7),'right',0.
    if fault=='epoch':
        scene=replace(scene,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in scene.rays))
    elif fault=='side':
        side='left'
    elif fault=='reset':
        observer.reset()
    elif fault=='stale':
        scene=replace(scene,t_ref_s=1.)
    else:
        speed=-.01
    assert not observer.evaluate(scene,history(),speed,side,.2,.02)['positive']


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('divergence',[.3,.5])
def test_diverging_walls_with_post_not_promoted_by_other_role(side,divergence):
    scene=cast_scene(.4,branch(divergence)+post(.36))
    result=module.current_proposal(scene,0.,side,0.,.02,config())
    assert not result['point_occupancy_supported'],result


@pytest.mark.parametrize('sign,side',[(1,'right'),(-1,'left')])
@pytest.mark.parametrize('kind',['center_block','missing_inner'])
def test_first_return_obstacle_or_missing_inner_remains_hold(sign,side,kind):
    scene=cast_scene(.4,(np.asarray(elbow(.30,kind))*[1,sign]).tolist())
    result=module.current_proposal(scene,0.,side,0.,.02,config())
    assert not result['point_occupancy_supported'],result


def test_both_steering_sweeps_keep_all_points(monkeypatch):
    scene=ring_scene(.15,1,.4)
    calls=[]
    def sweep(points,angle,*args,**kwargs):
        calls.append((np.array(points),np.array(kwargs['margin'])))
        return .10 if angle==.2 else 1.
    monkeypatch.setattr(module,'swept_limit',sweep)
    out=module.current_proposal(scene,0.,'right',.2,.02,config())
    assert out['geometry_supported'] and not out['point_occupancy_supported']
    assert len(calls)==2
    for points,padding in calls:
        np.testing.assert_array_equal(points,scene.occupied_endpoints_xy_m)
        assert len(padding)==len(points) and min(padding)>=.055
