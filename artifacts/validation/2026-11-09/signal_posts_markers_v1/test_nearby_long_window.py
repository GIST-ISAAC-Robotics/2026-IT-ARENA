"""후방 면과 같은 .8초 관측 후보: 반복/새 접근/가림은 근거를 대신하지 않는다."""
from dataclasses import replace
import numpy as np
from arena_autonomy.nearby_face_motion import NearbyFragments
from arena_autonomy.bypass_corridor_trial import CorridorPolicy
from test_nearby_face_motion import evaluate

def observer():
    return NearbyFragments(replace(CorridorPolicy(),window_s=.8,minimum_scans=9))

def settle(c):
    for t in np.linspace(.5,1.3,9): evaluate(c,float(t))

def test_eight_new_scans_or_many_reuses_are_not_long_evidence():
    c=observer()
    for t in np.linspace(.5,1.2,8):
        for _ in range(3):
            after,e=evaluate(c,float(t))
            assert e['changed']==0
    after,e=evaluate(c,1.3)
    assert e['changed']==36 and all(after[:-1]<.13) and after[-1]==.13

def test_new_approach_cannot_borrow_long_stationary_history():
    c=observer();settle(c)
    after,e=evaluate(c,1.4,advance=.04)
    assert all(after[:18]<.13) and all(after[18:]==.13)
    assert sorted(f['state']['stable'] for f in e['fragments'])==[False,True]

def test_long_history_does_not_authorize_reappeared_fragment():
    c=observer();settle(c)
    evaluate(c,1.4,missing=True)
    after,e=evaluate(c,1.5)
    assert all(after[:18]<.13) and all(after[18:]==.13)

def test_delayed_reappearance_clears_both_fragments():
    c=observer();settle(c)
    after,e=evaluate(c,1.7)
    assert e['changed']==0
