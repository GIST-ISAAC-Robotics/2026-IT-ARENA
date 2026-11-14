import importlib.util,json,sys
from pathlib import Path
import pytest
from arena_autonomy.pass_release_trial import SelectorConfig,motion_region,lateral_config,region_blocked
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/released_masked_speed_probe_v1'
import arena_autonomy.masked_motion_speed as m
REGION=dict(distance_m=.3,queries=12,unobserved=0,masked_left=6,masked_right=0,masked_tail_extent_m=.012,masked_front_samples=0)
PARTIAL=dict(allowed=True,reason='fully_observed')

@pytest.mark.parametrize('change',[{'unobserved':1},{'masked_front_samples':1},{'masked_tail_extent_m':.01},
    {'masked_tail_extent_m':.009},{'masked_left':0,'masked_right':0}])
def test_no_reinterpretation(change):
    assert m.lower_masked_motion(dict(REGION,**change),PARTIAL,.8,SelectorConfig(),lambda t:pytest.fail('unexpected retry')) is None

@pytest.mark.parametrize('partial',[{},dict(allowed=False,reason='fully_observed'),dict(allowed=True,reason='bounded_side_missing')])
def test_requires_original_observed_path(partial):
    assert m.lower_masked_motion(REGION,partial,.8,SelectorConfig(),lambda t:pytest.fail('unexpected retry')) is None

@pytest.mark.parametrize('target',[0.,.1,.15])
def test_never_raises_low_command(target):
    assert m.lower_masked_motion(REGION,PARTIAL,target,SelectorConfig(),lambda t:pytest.fail('unexpected retry')) is None

@pytest.mark.parametrize('works',[.35,.25,.15,None])
def test_bounded_descending_search(works):
    calls=[]
    def evaluate(t):
        calls.append(t)
        return dict(REGION,masked_tail_extent_m=.009 if t==works else .012),{'token':t}
    out=m.lower_masked_motion(REGION,PARTIAL,.8,SelectorConfig(),evaluate)
    assert calls==[.35,.25,.15][:([.35,.25,.15].index(works)+1) if works else 3]
    assert out['accepted']==(works is not None)
    if works:assert out['target_mps']==works and out['context']=={'token':works}
    assert REGION['masked_tail_extent_m']==.012

def test_reduced_region_must_still_be_observed():
    out=m.lower_masked_motion(REGION,PARTIAL,.8,SelectorConfig(),lambda t:(dict(REGION,masked_tail_extent_m=.005,unobserved=1),{}))
    assert not out['accepted']

def test_respects_smaller_existing_cap():
    calls=[]
    def evaluate(t):
        calls.append(t)
        return dict(REGION),{}
    out=m.lower_masked_motion(REGION,PARTIAL,.8,SelectorConfig(guard_speed_cap_mps=.1),evaluate)
    assert not out['accepted'] and calls==[.1]

@pytest.mark.parametrize('index',[0,1,2])
def test_saved_measured_motion_preserved(index):
    d=json.loads((A/'snapshots.json').read_bytes())[index]
    cfg=FollowConfig(**d['header']['follow_config']);mc=MotionConfig(**d['header']['motion_config'])
    i,scan,h=restore_call(d['input'],mc)
    scene=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    n=d['output'][2]['normal'];calls=[]
    def evaluate(target,measured=None):
        context={};measured=i['speed_mps'] if measured is None else measured
        calls.append(measured)
        r=motion_region(scene,measured,target,d['baseline_steering_rad'],i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg),context=context)
        return r,context
    out=m.lower_masked_motion(n['selector_region'],n['partial_observation'],d['baseline_target_mps'],SelectorConfig(),evaluate)
    assert out['accepted'] and out['target_mps']==.15 and all(v==i['speed_mps'] for v in calls)
    assert region_blocked(out['region'],SelectorConfig()) is None
    # 같은 장면에서 실제 속도가 높으면 낮은 명령을 냈다는 이유로 짧게 예측하지 않는다.
    high=m.lower_masked_motion(n['selector_region'],n['partial_observation'],d['baseline_target_mps'],SelectorConfig(),lambda t:evaluate(t,.9))
    assert not high['accepted']
