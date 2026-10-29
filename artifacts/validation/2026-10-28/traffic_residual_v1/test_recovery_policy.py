import math
from unittest.mock import patch
from recovery_policy import recovery_policy_class
from test_timed_policy_probe import points
from test_local_follow import scan,history

def setup():
    cls=recovery_policy_class();policy=cls(True,True,True);h=history()
    raw=scan(.4,[8.]*500);raw.ranges[375]=.15
    assert policy.evaluate(raw,points(h,raw,.5),h,.5,.15)['allowed']
    raw=scan(.5,[8.]*500);raw.ranges[375]=math.inf
    result=policy.evaluate(raw,points(h,raw,.6),h,.6,.15)
    assert not result['allowed']
    return policy,h,raw

def test_even_claimed_reobservation_cannot_skip_current_hidden_threat():
    p,h,raw=setup()
    with patch.object(p.anchor_probe,'observe',return_value=dict(matched=True,reason='test_claim',unknown_filled=False)):
        result=p.evaluate(raw,points(h,raw,.62),h,.62,.15)
    assert not result['allowed'] and not result['anchor_diagnostic']['released']
    assert result['anchor_diagnostic']['current_recheck_reason']=='known_obstacle_latched'

def test_wide_missing_does_not_reach_recovery_check():
    p,h,raw=setup();raw=scan(.6,[math.inf]*500)
    with patch.object(p.anchor_probe,'observe',side_effect=AssertionError('must not recover')):
        result=p.evaluate(raw,points(h,raw,.7),h,.7,.15)
    assert not result['allowed']

def test_truncated_evidence_stays_latched():
    p,h,raw=setup();p.latch_evidence['pairs']=len(p.latch_evidence['witnesses'])+1
    result=p.evaluate(raw,points(h,raw,.62),h,.62,.15)
    assert not result['allowed'] and result['anchor_diagnostic']['reason']=='incomplete_threat_set'

def test_original_clear_scans_still_need_two_fresh_sources():
    p,h,_=setup()
    raw=scan(.6,[8.]*500)
    assert not p.evaluate(raw,points(h,raw,.7),h,.7,.15)['allowed']
    assert not p.evaluate(raw,points(h,raw,.72),h,.72,.15)['allowed']
    raw=scan(.7,[8.]*500)
    assert p.evaluate(raw,points(h,raw,.8),h,.8,.15)['allowed']

def test_reset_clears_anchor_and_original_latch():
    p,_,_=setup();p.reset()
    assert not p.latched and p.anchor_probe.anchor is None and p.anchor_latch_time is None
