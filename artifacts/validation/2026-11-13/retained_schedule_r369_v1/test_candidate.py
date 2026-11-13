from types import SimpleNamespace
import pytest
from candidate import ScheduledProcess
from arena_autonomy.runtime_dual_state import RuntimeDualProcess


def make(**changes):
    obj=object.__new__(ScheduledProcess)
    plan=dict(generation=0,sequence=1,output=[.1,0,{'normal':{'selector_mode':'RELEASED'}}])
    obj.__dict__.update(avoid_superseded_retained=True,skipped_retained_submissions=0,closed=False,
        fault=None,saved_plan=None,plan=SimpleNamespace(inflight={}),current=SimpleNamespace(inflight=None),
        ready=None,retained=plan,generation=0,supported=lambda _:True)
    obj.__dict__.update(changes)
    return obj


def test_only_unstarted_retained_check_is_skipped(monkeypatch):
    monkeypatch.setattr(RuntimeDualProcess,'offer_next_current',lambda *a,**k:pytest.fail('base not needed'))
    worker=make(); before=worker.retained.copy()
    assert worker.offer_next_current({'reference_s':1},now_s=1) is False
    assert worker.skipped_retained_submissions==1 and worker.retained==before
    assert worker.plan.inflight=={} and worker.current.inflight is None


@pytest.mark.parametrize('case',['off','solo','new_plan','no_inflight_plan','running_current','ready','closed','fault','no_route','old_generation'])
def test_existing_contract_handles_all_other_cases(monkeypatch,case):
    seen=[]
    monkeypatch.setattr(RuntimeDualProcess,'offer_next_current',lambda *a,**k:seen.append(k) or 'existing')
    worker=make()
    if case=='off':worker.avoid_superseded_retained=False
    if case=='solo':worker.retained['output'][2]['normal']['selector_mode']='SOLO'
    if case=='new_plan':worker.saved_plan={}
    if case=='no_inflight_plan':worker.plan.inflight=None
    if case=='running_current':worker.current.inflight={}
    if case=='ready':worker.ready={}
    if case=='closed':worker.closed=True
    if case=='fault':worker.fault='worker failure'
    if case=='no_route':worker.retained=None
    if case=='old_generation':worker.generation=1
    assert worker.offer_next_current({'reference_s':1},now_s=1)=='existing'
    assert len(seen)==1 and worker.skipped_retained_submissions==0
