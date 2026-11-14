import pytest
from arena_autonomy.static_side_return import StaticSideTracker

def sequence(elapsed=True,object_speed=0.):
    tracker=StaticSideTracker();answers=[]
    for t in (0.,.2,.4,.6):
        if elapsed:tracker.advance_to(t,.5)
        else:tracker.advance(.5,.1)
        answers.append(tracker.observe(t,.7+(object_speed-.5)*t))
    return tracker,answers

def test_irregular_stationary_post_uses_elapsed_time():
    _,old=sequence(False)
    tracker,new=sequence(True)
    assert not any(old) and new[-1]
    assert tracker.travel_m==pytest.approx(.3)

def test_nominal_dt_can_mislabel_slow_moving_vehicle_as_fixed():
    _,old=sequence(False,.2)
    _,new=sequence(True,.2)
    assert old[-1] and not any(new)

@pytest.mark.parametrize('object_speed',[.5,1.,-.5])
def test_moving_object_not_released_by_elapsed_fix(object_speed):
    _,answers=sequence(True,object_speed)
    assert not any(answers)

def test_duplicate_reference_does_not_create_travel():
    tracker=StaticSideTracker()
    tracker.advance_to(1.,.5);tracker.advance_to(1.2,.5)
    before=tracker.travel_m
    for _ in range(10):tracker.advance_to(1.2,1.)
    assert tracker.travel_m==before and tracker.reference_speed==.5

def test_ramp_trapezoid():
    tracker=StaticSideTracker();tracker.advance_to(0.,0.);tracker.advance_to(.2,1.)
    assert tracker.travel_m==pytest.approx(.1)

@pytest.mark.parametrize('next_time',[.1,1.])
def test_discontinuity_discards_only_static_evidence(next_time):
    tracker=StaticSideTracker()
    tracker.advance_to(.2,.5);tracker.observe(.2,.5)
    tracker.advance_to(.4,.5);tracker.observe(.4,.4)
    tracker.advance_to(next_time,.5)
    assert tracker.travel_m==0 and tracker.rows==[] and tracker.reference_s==next_time

@pytest.mark.parametrize('speed',[-.01,float('nan'),float('inf')])
def test_invalid_or_reverse_speed_has_no_forward_credit(speed):
    tracker,_=sequence(True)
    tracker.advance_to(.8,speed)
    assert tracker.travel_m==0 and tracker.rows==[] and tracker.reference_s is None
