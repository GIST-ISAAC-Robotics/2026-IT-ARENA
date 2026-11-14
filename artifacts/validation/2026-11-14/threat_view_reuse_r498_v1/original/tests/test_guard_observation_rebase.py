import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import pytest
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles
from arena_autonomy import guard_observation_policy as candidate

A = Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-12/guard_approach_cap_r181_v1'


@pytest.fixture
def sample():
    path = A.parent/'adjacent_low_runtime_r180_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with path.open() as stream:
        header=json.loads(next(stream))['data']
        d=next(e['data'] for e in map(json.loads,stream) if e['kind']=='call'
               and e['data'].get('current_rebase',{}).get('output',{}).get('reason')=='guard_current_approach_not_observed')
    now=copy.deepcopy(d['current_rebase']['input'])
    _,scan,history=restore_call(now,MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,now['reference_s'],timing_verified=True,epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
    return d['output'],d['input'],now,scene,history,FollowConfig(**header['follow_config'])


def test_lower_target_requires_full_current_checks_with_source_and_inputs_preserved(sample):
    before=copy.deepcopy(sample[:3])
    base=candidate.common.rebase(*sample);result=candidate.rebase(*sample)
    assert base['speed_mps']==0
    assert 0 < result['speed_mps'] <= .25 < sample[0][0]
    assert result['current_approach_low_retry']['full_current_check']
    assert result['path_source_s']==base['path_source_s']
    assert equal_output(freeze(sample[:3]),freeze(before))
    assert result['independent_guard_required'] and not result['source_permission_reused']


@pytest.mark.parametrize('kind',['epoch','expired','fast_actual','near_front','missing_front','near_side'])
def test_bad_current_input_or_occupied_swept_path_does_not_gain_permission(sample,kind):
    out,old,now,scene,history,cfg=sample
    if kind=='epoch':now['epoch']+=1
    elif kind=='expired':out[2]['normal']['path_source_s']-=1.
    elif kind=='fast_actual':now['speed_mps']=.35
    else:
        _,scan,_=restore_call(now,history.config)
        angles=scan_angles(scan)+history.config.lidar_yaw_rad;values=list(scan.ranges_m)
        if kind=='near_side':
            for i,a in enumerate(angles):
                a=math.remainder(a,math.tau)
                if math.radians(65)<a<math.radians(135):values[i]=.07/math.sin(a)
        else:
            i=min(range(len(angles)),key=lambda i:abs(math.remainder(angles[i],math.tau)))
            values[i]=.05 if kind=='near_front' else math.inf
        scan=replace(scan,ranges_m=tuple(values));now['scan']['measurements']['ranges_m']=values
        scene=motion_scene(scan,history,now['reference_s'],timing_verified=True,epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
    result=candidate.rebase(out,old,now,scene,history,cfg)
    assert result['speed_mps']==0.


def test_successful_or_unrelated_rejection_does_not_run_extra_search(sample,monkeypatch):
    for answer in ({'speed_mps':.2,'reason':'ok'}, {'speed_mps':0.,'reason':'guard_current_new_side'}):
        calls=[]
        def original(*args):calls.append(1);return answer
        monkeypatch.setattr(candidate.common,'rebase',original)
        assert candidate.rebase(*sample) is answer and len(calls)==1


def test_zero_or_stop_remains_unsupported(sample):
    out=copy.deepcopy(sample[0]);out[0]=0.
    assert not candidate.supported(out) and candidate.rebase(out,None,None,None,None,None) is None
