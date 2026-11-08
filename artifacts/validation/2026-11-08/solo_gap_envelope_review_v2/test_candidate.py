"""저속 무반환 평가 반경 후보 반례. 운영 소스 미수정."""
import importlib.util,sys,math
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tests'))
P=ROOT/'artifacts/validation/2026-11-08/solo_gap_envelope_v1/candidate.py'
sp=importlib.util.spec_from_file_location('gap_candidate_review',P)
mod=importlib.util.module_from_spec(sp);sp.loader.exec_module(mod)
Candidate=mod.Candidate
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_partial_observation import sample
from test_local_follow import history

def ready(cls=Candidate,on=True):
    p=cls(True,True,True,oblique_metric_enabled=on);h=history()
    p.evaluate(sample(.4),[],h,.5,.5)
    return p,h

@pytest.mark.parametrize('indices',[range(301,308),range(193,200)])
@pytest.mark.parametrize('speed',[0.,.2,.4])
def test_smaller_body_and_stopping_envelope_can_progress(indices,speed):
    p,h=ready();raw=sample(.5,indices);before=list(raw.ranges)
    r=p.evaluate(raw,[],h,.6,speed)
    assert r['allowed'],r
    assert r['speed_cap_mps']==.2 and not r['unknown_filled']
    assert r['residual_hidden_object_risk']
    assert r['lateral_metric']['missing_chord_m']<=.05
    assert r['history_distance_m']>=.6
    np.testing.assert_equal(raw.ranges,before)

@pytest.mark.parametrize('speed',[0.,.2,.5,.6])
def test_off_is_original(speed):
    p,h=ready(on=False);q,j=ready(PartialObservationPolicy,False)
    raw=sample(.5,range(301,308))
    assert p.evaluate(raw,[],h,.6,speed)==q.evaluate(raw,[],j,.6,speed)

@pytest.mark.parametrize('case',['central','many','fast','stale','near','under','all','steer'])
def test_excessive_blind_or_front_or_bad_input_remains_blocked(case):
    p,h=ready();raw=sample(.5,range(301,308));v=.4;now=.6;kw={}
    if case=='central':raw=sample(.5,range(248,255))
    elif case=='many':raw=sample(.5,range(301,310))
    elif case=='fast':v=.61
    elif case=='stale':now=.81
    elif case=='near':raw.ranges[300]=.9
    elif case=='under':raw.ranges[304]=.01
    elif case=='all':raw=sample(.5,range(500))
    elif case=='steer':v=.6;kw={'steering':.5,'previous_steering':.5}
    assert not p.evaluate(raw,[],h,now,v,**kw)['allowed']

def test_speed_expands_radius_and_restores_rejection_before_trial_limit():
    out=[]
    for v in [0.,.2,.4,.5,.6]:
        p,h=ready();out.append(p.evaluate(sample(.5,range(301,308)),[],h,.6,v))
    r=[x['lateral_metric']['radius_m'] for x in out]
    assert r==sorted(r)
    assert out[0]['allowed'] and not out[-1]['allowed']
    assert out[-1]['lateral_metric']['missing_chord_m']>.05

@pytest.mark.parametrize('distance',[.16,.25,.4])
def test_disappearing_near_target_is_not_erased(distance):
    p,h=ready();angle=math.radians(39.)
    p._remember([[.06+distance*math.cos(angle),distance*math.sin(angle)]],.5,.4,True)
    r=p.evaluate(sample(.5,range(301,308)),[],h,.6,.4)
    assert not r['allowed'] and r['reason']=='known_obstacle_latched',r

@pytest.mark.parametrize('indices,w',[(range(301,308),4.),(range(193,200),-4.)])
def test_turning_into_front_is_not_a_side_gap(indices,w):
    from arena_autonomy.scene_motion import relative_poses
    p,h=ready();h=history(w=w)
    raw=sample(.5,indices);ids=np.array(list(indices))
    yaw=relative_poses(h,.5+ids*raw.time_increment,.7)[:,2]
    angles=raw.angle_min+ids*raw.angle_increment+yaw
    assert np.any(np.abs(angles)<math.radians(30))
    r=p.evaluate(raw,[],h,.7,.4)
    assert not r['allowed'],r
