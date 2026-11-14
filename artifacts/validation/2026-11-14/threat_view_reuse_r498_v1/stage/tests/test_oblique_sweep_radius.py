"""저속 무반환 평가 반경 후보 반례. 선택형 운영 연결 후 검사."""
import importlib.util,sys,math
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from arena_autonomy.partial_observation import PartialObservationPolicy as Candidate
import types,zipfile
_baseline=types.ModuleType('arena_autonomy.partial_before_r25')
_baseline.__package__='arena_autonomy'
with zipfile.ZipFile(ROOT/'artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/before.zip') as z:
    _old=z.read('src/arena_autonomy/arena_autonomy/partial_observation.py').decode()
exec(compile(_old,'partial_before_r25.py','exec'),_baseline.__dict__)
PartialObservationPolicy=_baseline.PartialObservationPolicy
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

@pytest.mark.parametrize('case',['many','fast','stale','near','under','all','steer'])
def test_excessive_blind_or_bad_input_remains_blocked(case):
    p,h=ready();raw=sample(.5,range(301,308));v=.4;now=.6;kw={}
    if case=='many':raw=sample(.5,range(301,310))
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
def test_turning_single_gap_uses_forward_metric_bound(indices,w):
    from arena_autonomy.scene_motion import relative_poses
    p,h=ready();h=history(w=w)
    raw=sample(.5,indices);ids=np.array(list(indices))
    yaw=relative_poses(h,.5+ids*raw.time_increment,.7)[:,2]
    angles=raw.angle_min+ids*raw.angle_increment+yaw
    assert np.any(np.abs(angles)<math.radians(30))
    r=p.evaluate(raw,[],h,.7,.4)
    # r69부터 단일 정면 공백도 같은 저속 이동 반경/50mm 폭으로 판정한다.
    # 회전 후30도 안으로 들어왔다는 이유만으로 무조건 거절하지 않는다.
    assert r['allowed'] and r['speed_cap_mps']==.2,r
    assert r['lateral_metric']['missing_chord_m']<=.05
    assert not r['unknown_filled']


def test_central_single_gap_is_bounded_low_speed_not_free_space():
    p,h=ready();raw=sample(.5,range(248,255))
    r=p.evaluate(raw,[],h,.6,.4)
    assert r['allowed'] and r['speed_cap_mps']==.2,r
    assert r['lateral_metric']['missing_chord_m']<=.05
    assert not r['unknown_filled']


@pytest.mark.parametrize('speed',[0.,.2,.4,.6])
@pytest.mark.parametrize('steer',[-.5,0.,.5])
def test_radius_contains_sampled_body_sweep_about_lidar(speed,steer):
    p,h=ready()
    r=p.evaluate(sample(.5,range(301,308)),[],h,.6,speed,steering=steer)
    radius=r['lateral_metric']['radius_m']
    distance=speed*.2+speed*speed/4+.06
    d=np.linspace(0,distance,61);k=math.tan(steer)/.135
    theta=d*k
    rear_x=d if abs(k)<1e-9 else np.sin(theta)/k
    rear_y=np.zeros_like(d) if abs(k)<1e-9 else (1-np.cos(theta))/k
    for x in [-.15,.15]:
        for y in [-.125,.125]:
            px=-.135/2+rear_x+(x+.135/2)*np.cos(theta)-y*np.sin(theta)-.06
            py=rear_y+(x+.135/2)*np.sin(theta)+y*np.cos(theta)
            assert np.max(np.hypot(px,py))<=radius+1e-12

def test_independent_guard_caps_progress_and_keeps_zero_command():
    import json
    from arena_autonomy.lidar_safety import LidarSafety
    from test_partial_observation import candidate_node
    node,commands,statuses=candidate_node(True)
    p,h=ready()
    node.partial_policy=p
    node.motion.history=h
    node.scan=sample(.5,range(301,308))
    node.command.drive.speed=.2
    LidarSafety.control(node)
    assert commands[-1].drive.speed==pytest.approx(.2)
    report=json.loads(statuses[-1].data)
    assert report['partial_observation']['allowed']
    assert not report['partial_observation']['unknown_filled']
    node.command.drive.speed=0.
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.
