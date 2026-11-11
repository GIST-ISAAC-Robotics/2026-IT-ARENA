"""후방 구조 기억 위임의 범위와 실제 기억 제동을 검사한다. 런타임에 연결하지 않는다."""
import importlib.util,json,math
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_known_structure_policy import masked
from test_local_follow import history

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r77_candidate',HERE/'known_structure_candidate.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)


def make_case(point=(-.3,.5), stored=True, flank=True, speed=.2, steering=0.):
    sectors=[[126.,4.]]
    raw=masked(sectors=sectors)
    if not flank:
        _,_,rows=candidate.classify(raw,sectors)
        raw.ranges[rows[0]['flanks'][0]]=float('nan')
    valid,_,rows=candidate.classify(raw,sectors)
    p=candidate.structure_policy(PartialObservationPolicy,json.dumps(sectors))()
    points=np.array([point],dtype=float)
    if stored:p.structure_previous=(points.copy(),.5)
    p.structure_active_call=(raw,valid,rows,[],history(),.6,.5,speed,steering,steering,.135,.2,.15)
    report={};threat=np.ones((2,1),dtype=bool)
    output=p._refine_threat(threat,points,report)
    return p,output,report,threat,raw


def test_existing_rear_memory_handles_same_point_without_deleting_unknown_or_memory():
    p,output,report,original,raw=make_case()
    assert not output.any() and original.all()
    assert report['structure_rear_memory_transfer']['points']==1
    assert len(p.structure_hidden[0])==1
    assert p.structure_precomputed['hidden_memory_points']==1
    assert np.isnan(raw.ranges).any()


@pytest.mark.parametrize('kwargs',[dict(point=(.3,.5)),dict(point=(-.3,-.5)),dict(stored=False),dict(flank=False)])
def test_front_unexplained_or_unremembered_point_is_not_delegated(kwargs):
    _,output,report,_,_=make_case(**kwargs)
    assert output.all()
    assert 'structure_rear_memory_transfer' not in report


def test_turning_back_toward_retained_rear_point_still_limits_speed():
    p,output,_,_,_=make_case(speed=2.4,steering=.37)
    assert not output.any()
    assert p.structure_precomputed['memory_cap_mps'] is not None
    assert p.structure_precomputed['memory_cap_mps']<2.4


def test_structure_evaluation_is_reused_once_in_full_policy_call(monkeypatch):
    from test_known_structure_policy import policy
    # 기본 구조 정책의 기존 전체 검사들을 후보 mixin에 연결한다.
    import arena_autonomy.known_structure as runtime
    monkeypatch.setattr(runtime,'StructureAwareMixin',candidate.StructureAwareMixin)
    p=policy([[126.,4.]])
    calls=[];method=p._structure
    def wrapped(*args,**kwargs):
        calls.append(1);return method(*args,**kwargs)
    monkeypatch.setattr(p,'_structure',wrapped)
    raw=masked(sectors=[[126.,4.]])
    raw.ranges[375]=float('nan')  # 구조 밖90도 공백. 과거 뒤쪽 점의 이동 확장과 겹친다.
    point=np.array([[-.3,.5]])
    p._remember(point,.4,.3,True)
    p.structure_previous=(point.copy(),.4)
    r=p.evaluate(raw,[],history(),.6,.2)
    assert r['allowed'] and len(calls)==1
    assert r['structure_rear_memory_transfer']['points']==1


def test_early_structure_failure_keeps_threat_and_returns_existing_error(monkeypatch):
    from arena_autonomy.lidar_motion import MotionUnavailable
    p=candidate.structure_policy(PartialObservationPolicy,json.dumps([[126.,4.]]))(
        allow_narrow_forward=True,surface_translation_enabled=True,lateral_metric_enabled=True)
    def unavailable(*args,**kwargs):
        raise MotionUnavailable('stored structure history unavailable')
    monkeypatch.setattr(p,'_structure',unavailable)
    raw=masked(sectors=[[126.,4.]])
    raw.ranges[375]=float('nan')
    point=np.array([[-.3,.5]])
    p._remember(point,.4,.3,True)
    p.structure_previous=(point.copy(),.4)
    result=p.evaluate(raw,[],history(),.6,.2)
    assert not result['allowed'] and result['reason']=='structure_motion_unavailable'
    assert 'structure_rear_memory_transfer' not in result
    assert p.structure_active_call is None
