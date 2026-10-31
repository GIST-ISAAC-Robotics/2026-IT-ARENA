from observer_v3 import RejoinYieldObserver
from test_observer import row


def active():
    o=RejoinYieldObserver()
    for i in range(9):o.update(row(i))
    assert o.detail['suggest_yield']
    return o


def test_one_clear_band_does_not_accelerate_before_existing_guide_confirmation():
    o=active()
    for i in (9,10):
        r=row(i);r.update(band_supported=True,blocked_by_current_returns=False)
        d=o.update(r)
        assert d['suggest_yield'] and d['current_band_supported'] and not d['control_authorized']
    r=row(11);r.update(phase='rejoin')
    assert not o.update(r)['suggest_yield']


def test_new_clear_band_cannot_start_yield():
    o=RejoinYieldObserver()
    for i in range(12):
        r=row(i);r.update(band_supported=True,blocked_by_current_returns=False)
        assert not o.update(r)['suggest_yield']


def test_clear_band_does_not_extend_budget_without_guide_transition():
    o=active()
    for i in range(9,57):
        r=row(i);r.update(band_supported=True,blocked_by_current_returns=False)
        assert o.update(r)['suggest_yield']
    r=row(57);r.update(band_supported=True,blocked_by_current_returns=False)
    assert not o.update(r)['suggest_yield']
    assert o.detail['reason']=='continuation_budget_expired'


def test_missing_or_changed_observation_still_clears_cap():
    o=active();assert not o.update(None)['suggest_yield']
    o=active();r=row(9);r['normal']=.5
    assert not o.update(r)['suggest_yield']
