"""같은 이동 영역 밖의 후방 공백만 분리한다. 원 점유/앞부분/실제 후방 질의는 유지."""
from pathlib import Path
import importlib.util,math
import numpy as np
import pytest
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap as original
from test_same_path_side_scope import case
from test_local_follow import history

spec=importlib.util.spec_from_file_location('staged_query_local_side',Path(__file__).with_name('same_path_gap_trial.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
candidate=module.bounded_same_path_gap

def build(degrees=54,extra=None):
    raw,_,proof,command,q=case(degrees)
    raw.ranges[0]=math.inf
    for k,v in (extra or {}).items():raw.ranges[k]=v
    scene=motion_scene(raw,history(),.6,timing_verified=True,rear_half_angle_deg=0.)
    return scene,raw,q,np.full(500,.1),.2,.15,proof,command

@pytest.mark.parametrize('degrees',[-90,-54,45,54,90])
def test_unqueried_rear_seam_does_not_cancel_same_side_motion(degrees):
    args=build(degrees);before=tuple(args[1].ranges)
    assert original(*args)['reason']=='unknown_outside_side_scope'
    result=candidate(*args)
    assert result['accepted'] and result['query_local_side_scope']
    assert not result['unknown_filled'] and result['unqueried_rear_unknown_count']==1
    assert tuple(args[1].ranges)==before and args[0].rays[0].status=='unknown'

@pytest.mark.parametrize('degrees',[-120,-101,-30,0,30,101,120])
def test_queried_front_or_rear_gap_cannot_borrow_side_permission(degrees):
    assert not candidate(*build(degrees))['accepted']

def test_unqueried_front_gap_still_contradicts_side_only_proof():
    assert not candidate(*build(extra={250:math.inf}))['accepted']

@pytest.mark.parametrize('extra',[
    {324:.08},{325:float('nan')},{325:-math.inf},{325:0.},
    {i:math.inf for i in range(322,329)},
])
def test_near_invalid_and_wide_queried_gaps_remain_rejected(extra):
    assert not candidate(*build(extra=extra))['accepted']

@pytest.mark.parametrize('change',[{'steering':.21},{'previous':.21},{'target':.36},{'speed':.4},{'now':.61}])
def test_scope_belongs_to_current_checked_command(change):
    args=list(build());args[-1].update(change)
    assert not candidate(*args)['accepted']

@pytest.mark.parametrize('degrees',[-120,-30,0,54,90,120])
def test_without_added_rear_gap_keeps_exact_old_result(degrees):
    raw,scene,p,c,q=case(degrees)
    args=scene,raw,q,np.full(500,.1),.2,.15,p,c
    assert candidate(*args)==original(*args)
