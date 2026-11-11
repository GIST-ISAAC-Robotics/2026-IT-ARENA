import copy
import json
from pathlib import Path
from unittest.mock import patch

import pytest

import current
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene

A=Path(__file__).resolve().parent


def fixture():
    return json.loads((A/'fixtures.json').read_bytes())[0]


def run(data):
    owner,motion=make_controller(data['header'])
    inputs,scan,history=restore_call(data['current'],motion)
    scene=motion_scene(scan,history,inputs['reference_s'],timing_verified=inputs['timing_verified'],
        epoch=inputs['epoch'],rear_half_angle_deg=inputs['rear_half_angle_deg'])
    return current.rebase(data['original']['output'],data['original']['input'],inputs,scan,scene,history,owner.config)


def test_saved_current_result_is_rechecked_without_input_mutation():
    data=fixture();before=copy.deepcopy(data)
    with patch.object(current.adjacent,'bounded_adjacent_structure',wraps=current.adjacent.bounded_adjacent_structure) as check:
        out=run(data)
        assert check.call_count==1
    assert equal_output(freeze(out),data['output']) and data==before
    assert out['speed_mps']>0 and out['current_gap_proof']['policy']=='guard_adjacent_structure_gap_r149'
    assert not out['source_permission_reused'] and not out['current_gap_proof']['unknown_filled']


@pytest.mark.parametrize('change',['epoch','expiry','steering','speed','front','mask_gap','close_outer'])
def test_current_change_rejects_old_permission(change):
    data=fixture();raw=data['current']['scan']['measurements']
    if change=='epoch':data['current']['epoch']+=1
    elif change=='expiry':data['original']['output'][2]['normal']['path_source_s']=data['current']['reference_s']-.30001
    elif change=='steering':data['current']['previous_steering_rad']+=.1
    elif change=='speed':data['current']['speed_mps']=.3
    elif change=='front':
        import math
        for i,a in enumerate(raw['angles_rad']):
            if abs((a+math.pi)%(2*math.pi)-math.pi)<.2:raw['ranges_m'][i]=.12
    else:
        group=data['output']['current_gap_proof']['gaps'][0]
        if change=='close_outer':raw['ranges_m'][group['last']+1]=.2
        else:
            unknown=next(i for i in range(group['first'],group['last']+1) if i not in group['masked_indices'])
            raw['ranges_m'][unknown]={'nonfinite':'nan'}
    assert run(data)['speed_mps']==0


def test_new_side_object_is_not_released():
    original=current.ps.threat_view
    def occupied(*args):
        view=original(*args);view['left']['present']=True;return view
    with patch.object(current.ps,'threat_view',occupied):
        out=run(fixture())
        assert out['speed_mps']==0 and out['reason']=='guard_rebase_current_side_present'


def test_unrecognized_old_policy_cannot_borrow_new_risk():
    data=fixture()
    data['original']['output'][2]['normal']['selector_guard_candidate']['same_path_gap']['policy']='unknown_policy'
    assert run(data)['speed_mps']==0
