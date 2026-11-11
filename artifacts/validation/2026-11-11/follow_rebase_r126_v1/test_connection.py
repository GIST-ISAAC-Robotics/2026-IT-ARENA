from types import SimpleNamespace as NS
from copy import deepcopy
import pytest

pytest.importorskip('rclpy')
import trial_command as target


def fixture(kind='candidate'):
    emitted=[]
    data=dict(reference_s=1.1,epoch=0,side='left',previous_steering_rad=.1)
    output=[.3,.1,dict(reason='handoff_follow',handoff_state='FOLLOW',normal=dict(
        reason='parallel_hold_follow',selector_mode='GUARD',selector_choice='parallel'))]
    item=dict(kind=kind,input=data,output=output,reference_s=1.1,source_age_s=.2,
        result=dict(output=output,completed_wall_s=2.),reason='decision_source_expired')
    node=NS(decision_delivery=NS(poll=lambda *a,**kw:deepcopy(item),counts={}),
        now_s=lambda:1.2,last_steering=.1,side='left',scan=NS(source_epoch=0),
        motion=NS(counts={'resets':0},last_meta={'reference_stamp_s':1.2},
            history=NS(wheels=NS(values=[.2])),verified=True),
        follow_trace=NS(emit=lambda kind,data:emitted.append((kind,data))),
        follow=NS(config=None,reset=lambda:None),prepared_output=lambda output:output,
        decision_output_dt_s=.1,rear_blind_half_angle=30.)
    return node,item,emitted


def test_guard_result_preserves_original_stamp_and_raw_evidence():
    node,item,events=fixture()
    result=target.command(node,[])
    assert result[:2]==(.3,.1) and node.decision_command_stamp_s==1.1
    assert 'current_rebase' not in events[0][1]
    assert events[0][1]['output']==item['output']


def test_discarded_delivery_cannot_be_rebased():
    node,item,events=fixture('discarded')
    assert target.command(node,[]) is None
    assert [k for k,d in events]==['call','reset']
    assert not hasattr(node,'decision_command_stamp_s')


def test_completed_positive_result_records_current_recalculation_not_rewritten_original(monkeypatch):
    node,item,events=fixture()
    item['output'][2]['normal'].update(reason='local_follow',selector_mode='SOLO',selector_choice='baseline')
    current=dict(reference_s=1.2)
    monkeypatch.setattr(target,'snapshot_call',lambda *a,**kw:current)
    monkeypatch.setattr(target,'motion_scene',lambda *a,**kw:object())
    monkeypatch.setattr(target,'rebase',lambda *a,**kw:dict(speed_mps=.2,steering_rad=.12,
        reference_s=1.2,reason='rebase_current_follow',path_source_s=1.,independent_guard_required=True))
    result=target.command(node,[])
    assert result[:2]==(.2,.12) and node.decision_command_stamp_s==1.2
    assert events[0][1]['input']['reference_s']==1.1 and events[0][1]['output'][0]==.3
    assert events[0][1]['current_rebase']['input']['reference_s']==1.2
    assert events[0][1]['current_rebase']['output']['speed_mps']==.2
