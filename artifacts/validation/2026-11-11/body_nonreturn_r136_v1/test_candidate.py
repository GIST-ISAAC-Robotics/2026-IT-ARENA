from copy import deepcopy
from types import SimpleNamespace
import math
import numpy as np
import pytest
import candidate as c
from test_rear_fringe_trial import case


def probe(*,point=(.02,.08),missing=(411,412),speed=0.,target=.35,**kwargs):
    _,raw,scene=case(point=point,missing=missing,**kwargs)
    original=np.asarray(raw.ranges).copy()
    result=c.bounded_body_nonreturn(scene,raw,[point],np.full(500,.1),.2,.15,speed,target)
    np.testing.assert_array_equal(raw.ranges,original)
    return result,raw,scene


def test_body_front_half_can_be_rear_lidar_sector_without_being_free():
    result,raw,scene=probe()
    assert result['accepted'] and result['exterior_max_m']==pytest.approx(.005)
    assert not result['unknown_filled'] and result['residual_hidden_object_risk']
    assert result['maximum_chord_m']<=.05
    assert math.isinf(raw.ranges[411]) and scene.rays[411].status=='unknown'


@pytest.mark.parametrize('kwargs',[
    dict(speed=.351),dict(speed=-.01),dict(speed=math.nan),dict(target=.351),dict(target=0.),
    dict(point=(.101,.08)),dict(point=(-.101,.08)),dict(point=(.02,.085001)),
    dict(point=(.02,.075)),dict(value=math.nan),dict(value=-math.inf),dict(value=.02),
    dict(neighbor=.4),dict(neighbor=.1),dict(mismatch=True),dict(missing=tuple(range(407,418))),
    dict(point=(.05,.08),missing=(384,385))])
def test_fast_reverse_wide_front_close_invalid_inputs_still_block(kwargs):
    assert not probe(**kwargs)[0]['accepted']


@pytest.mark.parametrize('point,missing',[
    ((-.047,.0765),(451,452)),((-.047,-.0765),(48,49)),((.02,-.08),(87,88))])
def test_both_sides_and_old_rear_half_have_same_scope(point,missing):
    assert probe(point=point,missing=missing)[0]['accepted']


def selection(monkeypatch,*,mode='GUARD',present=False,allowed=True,enabled=True):
    monkeypatch.setattr(c,'ORIGINAL',lambda *args:dict(accepted=False,reason='partial_not_allowed'))
    _,raw,scene=probe()
    owner=SimpleNamespace(mode=mode,follow_config=SimpleNamespace(traffic_recovery_trial=enabled,
        length_m=.2,width_m=.15),sides={'left':{'present':present,'passed':False,'vanish':'unknown'}})
    original=deepcopy(owner.sides)
    context=dict(queries=np.array([[.02,.08]]),padding=np.full(500,.1),free=np.array([False]),masked=np.array([False]))
    result=c.same_path_with_body(owner,scene,raw,dict(unobserved=1),context,0.,.35,.2,.2,
        dict(partial_observation=dict(allowed=allowed,reason='fully_observed')))
    assert owner.sides==original
    assert not context['free'][0]
    return result


def test_guard_unknown_state_is_retained(monkeypatch):
    assert selection(monkeypatch)['accepted']


@pytest.mark.parametrize('kwargs',[dict(mode='SOLO'),dict(mode='RELEASED'),dict(present=True),dict(allowed=False),dict(enabled=False)])
def test_no_permission_to_expand_mode_or_present_vehicle(monkeypatch,kwargs):
    assert not selection(monkeypatch,**kwargs)['accepted']


def test_explicit_spawn_worker_starts_computes_and_exits(monkeypatch):
    import json
    from pathlib import Path
    import time
    import arena_autonomy.decision_process as dp
    trace=Path(__file__).resolve().parent.parent/'parallel_rebase_r133_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header=json.loads(next(stream))['data']
        first=next(json.loads(line) for line in stream if json.loads(line)['kind']=='call')
    monkeypatch.setattr(dp,'_worker',c.worker_entry)
    process=dp.DecisionProcess(header)
    try:
        process.offer(first['data']['input'])
        deadline=time.monotonic()+5.
        result=None
        while result is None and time.monotonic()<deadline:
            result=process.pump();time.sleep(.005)
        assert result is not None and process.fault is None
    finally:
        stopped=process.close()
    assert stopped==dict(forced=False,exitcode=0)
