import copy
import json
from pathlib import Path
from unittest.mock import patch

import pytest
from priority import CurrentPriorityProcess,may_finish_current,OverlapProcess

A=Path(__file__).resolve().parent


@pytest.fixture
def records():
    path=A.parent/'current_region_runtime_r282_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with path.open() as f:
        for line in f:
            e=json.loads(line)
            if e['kind']=='call' and 20<e['data']['input']['reference_s']<20.4:
                c=e['data'];break
    old=dict(stage='plan',generation=0,sequence=1,input=c['input'],output=c['output'])
    old['route_check']=dict(revision=8,retained=True)
    new=copy.deepcopy(old);new['sequence']=3;new['input']['reference_s']+=.1
    current=dict(stage='current',generation=0,sequence=2,plan=copy.deepcopy(old),
                 input=c['current_rebase']['input'],output=c['current_rebase']['output'])
    return old,new,current


def worker(old):
    w=CurrentPriorityProcess.__new__(CurrentPriorityProcess)
    w.current_input=dict(plan=old);w.ready=None;w.saved_plan=None;w.retained=old
    w.fault=None;w.revision=8;w.generation=0;w.last_application=-float('inf')
    w.last_checked_source=None;w.reset_reason=None
    return w


def collect(w,rows):
    def base(owner):
        assert owner is w
        for row in rows:
            if row['stage']=='plan':w.saved_plan=copy.deepcopy(row)
        w.current_input=None
        return rows
    with patch.object(OverlapProcess,'collect',side_effect=base):
        return w.collect()


def test_same_positive_plan_keeps_one_current_result_then_new_plan(records):
    old,new,current=records;w=worker(old)
    rows=collect(w,[current,new])
    assert rows[1]['deferred_positive_transition']
    assert w.revision==8 and w.ready['sequence']==2 and w.saved_plan['sequence']==3


@pytest.mark.parametrize('change', ['zero','slower','handoff','selector','wall','epoch','source','side','earlier','nan'])
def test_changed_intent_or_input_boundary_preempts(records,change):
    old,new,current=records;w=worker(old)
    if change=='zero':new['output'][0]=0
    if change=='slower':new['output'][0]=old['output'][0]/2
    if change=='handoff':new['output'][2]['handoff_state']='BYPASS'
    if change=='selector':new['output'][2]['normal']['selector_mode']='GUARD'
    if change=='wall':new['output'][2]['normal']['path_wall']='right'
    if change=='epoch':new['input']['epoch']+=1
    if change=='source':new['input']['scan']['measurements']['source_epoch']+=1
    if change=='side':new['input']['side']='right'
    if change=='earlier':new['input']['reference_s']=old['input']['reference_s']-.1
    if change=='nan':new['output'][0]=float('nan')
    assert not may_finish_current(old,new)
    rows=collect(w,[current,new])
    assert w.ready is None and rows[0]['discard_reason']=='new_plan_superseded_current_check'


def test_stop_between_deferred_plan_and_apply_clears_both(records):
    old,new,current=records;w=worker(old);collect(w,[new,current])
    w.invalidate('explicit_stop')
    assert w.ready is None and w.retained is None and w.saved_plan is None


def test_no_active_check_uses_new_revision(records):
    old,new,_=records;w=worker(old);w.current_input=None
    collect(w,[new])
    assert w.revision==9 and w.saved_plan['sequence']==3


def test_inflight_retained_failure_never_becomes_permission(records):
    old,new,current=records;w=worker(old);current['output']['speed_mps']=0
    rows=collect(w,[new,current])
    assert w.ready is None and rows[1]['discard_reason']=='retained_route_not_proven'
