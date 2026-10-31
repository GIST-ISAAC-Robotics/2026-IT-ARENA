import pytest
from observer import RelativeYieldObserver


def row(i,retreat=0.):
    return dict(phase='entry',side='right',epoch=1,source=1+i*.1,now=1.15+i*.1,heading=0.,speed=.35,
        normal=.35,along_low=.1-retreat*i*.1,along_high=.3-retreat*i*.1,
        method='parallel_face_median',blocked_by_current_returns=True,band_supported=False,wall_confirmations=3)


def test_same_speed_side_face_requests_lower_speed_but_not_permission():
    observer=RelativeYieldObserver()
    for i in range(8):assert not observer.update(row(i))['suggest_yield']
    result=observer.update(row(8))
    assert result['suggest_yield'] and result['trial_speed_cap_mps']==.2
    assert not result['control_authorized'] and not result['moving_object_identity_proven']


def test_stationary_obstacle_retreating_as_ego_passes_does_not_trigger():
    observer=RelativeYieldObserver()
    for i in range(9):assert not observer.update(row(i,.35))['suggest_yield']


def test_reused_scan_cannot_build_window():
    observer=RelativeYieldObserver()
    for _ in range(50):assert not observer.update(row(0))['suggest_yield']
    assert len(observer.rows)==1


@pytest.mark.parametrize('key,value',[('phase','rejoin'),('method','low_order'),('speed',-.1),('speed',float('nan')),
    ('now',3.),('heading',.2),('wall_confirmations',1),('blocked_by_current_returns',False),('band_supported',True)])
def test_ineligible_current_evidence_clears_advice(key,value):
    observer=RelativeYieldObserver()
    for i in range(9):observer.update(row(i))
    assert observer.detail['suggest_yield']
    bad=row(9);bad[key]=value
    assert not observer.update(bad)['suggest_yield']


@pytest.mark.parametrize('kind',['epoch','gap','backwards','side','different_face'])
def test_discontinuous_or_different_observations_do_not_inherit_window(kind):
    observer=RelativeYieldObserver()
    for i in range(8):observer.update(row(i))
    new=row(8)
    if kind=='epoch':new['epoch']=2
    if kind=='gap':new.update(source=3.,now=3.15)
    if kind=='backwards':new.update(source=1.2,now=1.35)
    if kind=='side':new['side']='left'
    if kind=='different_face':new.update(along_low=.45,along_high=.65)
    assert not observer.update(new)['suggest_yield']


def test_changing_visible_width_is_not_stable_face():
    observer=RelativeYieldObserver()
    for i in range(9):
        new=row(i);new['along_high']+=.02*i
        result=observer.update(new)
    assert not result['suggest_yield']
