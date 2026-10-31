import pytest
from observer_v2 import PersistentYieldObserver
from test_observer import row


def active():
    observer=PersistentYieldObserver()
    for i in range(9):observer.update(row(i))
    assert observer.detail['suggest_yield']
    return observer


def test_observed_face_can_move_forward_without_renewing_activation_budget():
    o=active();since=o.yield_since
    for i in range(9,40):
        r=row(i);shift=.015*(i-8);r.update(along_low=.1+shift,along_high=.3+shift)
        if i>30:r['blocked_by_current_returns']=False
        d=o.update(r)
        assert d['suggest_yield'] and not d['control_authorized'] and not d['unknown_filled']
    assert o.yield_since==since and r['along_low']>.5


def test_far_face_cannot_initiate_yield():
    o=PersistentYieldObserver()
    for i in range(15):
        r=row(i);r.update(along_low=.6,along_high=.8)
        assert not o.update(r)['suggest_yield']


@pytest.mark.parametrize('change',[
    dict(band_supported=True),dict(phase='rejoin'),dict(wall_confirmations=0),
    dict(epoch=2),dict(side='left'),dict(normal=.5),dict(speed=-.1),
    dict(heading=.1),dict(source=3.,now=3.1),dict(along_low=.8,along_high=1.),
])
def test_continuation_ends_on_current_fault_clear_band_or_new_face(change):
    o=active();r=row(9);r.update(change)
    assert not o.update(r)['suggest_yield']


def test_continuation_cannot_last_indefinitely_or_gain_time_by_scan_reuse():
    o=active();since=o.yield_since
    for i in range(9,57):assert o.update(row(i))['suggest_yield']
    assert o.yield_since==since
    assert not o.update(row(57))['suggest_yield']
    assert o.detail['reason']=='continuation_budget_expired'


def test_lost_observation_clears_continuation():
    o=active();assert not o.update(None)['suggest_yield']
