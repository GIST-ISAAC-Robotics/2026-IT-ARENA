import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import pytest
import released
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles

A=Path(__file__).resolve().parent


@pytest.fixture
def sample():
    trace=A.parent/'path_side_runtime_r165_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as f:
        header=json.loads(next(f))['data']
        item=next(r['data'] for r in map(json.loads,f) if r['kind']=='call' and released.supported(r['data']['output']))
    old=item['input'];current=copy.deepcopy(old)
    values,scan,history=restore_call(current,MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,current['reference_s'],timing_verified=True,epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    return item['output'],old,current,scan,scene,history,FollowConfig(**header['follow_config'])


def test_current_recalculation_preserves_source_age_state_and_speed_ceiling(sample):
    out,old,now,scan,scene,history,cfg=sample
    before=copy.deepcopy(out)
    r=released.rebase(out,old,now,scene,history,cfg)
    assert r['reason']=='rebase_current_released' and 0<r['speed_mps']<=out[0]
    assert r['path_source_s']==out[2]['normal']['path_source_s']
    assert not r['state_transition'] and not r['source_permission_reused']
    assert out==before


@pytest.mark.parametrize('kind',['epoch','sensor_epoch','side','steering','path_age'])
def test_binding_and_memory_limits_remain(sample,kind):
    out,old,now,scan,scene,history,cfg=sample
    if kind=='epoch':now['epoch']+=1
    elif kind=='sensor_epoch':now['scan']['measurements']['source_epoch']+=1
    elif kind=='side':now['side']='right' if now['side']=='left' else 'left'
    elif kind=='steering':now['previous_steering_rad']+=.1
    else:out[2]['normal']['path_source_s']-=1.
    assert released.rebase(out,old,now,scene,history,cfg)['speed_mps']==0


@pytest.mark.parametrize('kind',['present','not_passed','missing_state'])
def test_unreleased_or_ambiguous_original_side_is_outside_scope(sample,kind):
    out,old,now,scan,scene,history,cfg=sample
    side=out[2]['normal']['selector_sides']['left']
    if kind=='present':side['present']=True
    elif kind=='not_passed':side['passed']=False
    else:del side['seen']
    assert not released.supported(out)
    assert released.rebase(out,old,now,scene,history,cfg) is None


@pytest.mark.parametrize('kind',['close_front','missing_front','close_side'])
def test_new_current_scan_threat_is_not_hidden_by_old_release(sample,kind):
    out,old,now,scan,scene,history,cfg=sample
    angles=scan_angles(scan)+history.config.lidar_yaw_rad
    values=list(scan.ranges_m)
    if kind=='close_side':
        for i,a in enumerate(angles):
            a=math.remainder(a,math.tau)
            if math.radians(70)<a<math.radians(130):values[i]=.13/math.sin(a)
    else:
        i=min(range(len(angles)),key=lambda i:abs(math.remainder(angles[i],math.tau)))
        values[i]=math.inf if kind=='missing_front' else .05
    scan=replace(scan,ranges_m=tuple(values))
    scene=motion_scene(scan,history,now['reference_s'],timing_verified=True,epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
    # 현재 snapshot도 같은 실측 배열로 맞춘다. 과거 입력은 그대로다.
    now['scan']['measurements']['ranges_m']=values
    r=released.rebase(out,old,now,scene,history,cfg)
    assert r['speed_mps']==0
