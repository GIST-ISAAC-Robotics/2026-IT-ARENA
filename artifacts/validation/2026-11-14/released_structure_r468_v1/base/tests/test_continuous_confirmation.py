"""관측 경로 샘플 간격과 시간 확인을 구분하는 직각 후보 회귀."""
from types import SimpleNamespace
import numpy as np
import pytest
import arena_autonomy.corridor_observer as module
from arena_autonomy.local_follow import FollowConfig,path_agrees
from test_local_follow import history


def path(shift=0.):
    x=np.linspace(.06,.34,7)+shift
    return np.c_[x,.2*x]


def test_actual_common_length_survives_different_sampling():
    a,b=path(),path(.001)
    assert not path_agrees(a,b,.08)
    assert module.path_agrees_common_segments(a,b,.08)
    assert module.path_agrees_common_segments(b,a,.08)


@pytest.mark.parametrize('kind',['lateral','short','few','reverse','fold','far','nonfinite','narrow_overlap'])
def test_common_segment_rejects_unrelated_or_insufficient_paths(kind):
    a,b=path(),path(.001)
    if kind=='lateral':b[:,1]+=.09
    if kind=='short':b=b[:5]
    if kind=='few':b=b[::2]
    if kind=='reverse':b=b[::-1]
    if kind=='fold':b[3,0]=b[1,0]
    if kind=='far':b[:,0]+=1.
    if kind=='nonfinite':b[2,1]=float('nan')
    if kind=='narrow_overlap':b[:,0]+=.10
    assert not module.path_agrees_common_segments(a,b,.08)


@pytest.mark.parametrize('enabled,short',[(True,True),(False,True),(True,False)])
def test_temporal_confirmation_needs_distinct_scans_and_explicit_short_trial(monkeypatch,enabled,short):
    cfg=FollowConfig(piecewise_wall_trial=enabled,short_boundary_prefix_enabled=True,
                     corridor_fallback_enabled=True,boundary_evidence_mode='corner_distance_candidate',
                     occupancy_timing_mode='per_ray_candidate',solo_route_only=True,corridor_arc_agreement=True)
    observer=module.CorridorObserver(cfg);state={'path':path(),'occupied':True,'geometry':True}
    def proposal(*a,**kw):
        return dict(geometry_supported=state['geometry'],geometry={'path':state['path']},
            point_occupancy_supported=state['occupied'],short_prefix=short,
            reason='geometry_and_point_occupancy_only',proposed_speed_mps=.35,steering_rad=0.)
    monkeypatch.setattr(module,'current_proposal',proposal)
    def call(t,source):
        scene=SimpleNamespace(t_ref_s=t,source_first_s=source,source_frame_id='laser_frame',source_epoch=0)
        return observer.evaluate(scene,history(),0.,'left',0.,.02)
    assert not call(.4,.3)['positive']
    for t in (.42,.44,.46):
        r=call(t,.3)
        assert r['confirmations']==1 and not r['positive']
    state['path']=path(.001)
    assert not call(.5,.4)['positive']
    state['path']=path()
    result=call(.6,.5)
    assert result['positive']==bool(enabled and short)
    if enabled and short:
        assert result['path_agreement_method']=='continuous_common_x'
        assert result['confirmations']==3 and result['release_count']==2
    # 현 점유 거절·기하 소실은 이전의 일치로 덮지 않는다.
    state['occupied']=False
    result=call(.62,.5)
    assert not result['positive'] and result['release_count']==0
    state['geometry']=False
    assert not call(.7,.6)['positive'] and observer.path is None
