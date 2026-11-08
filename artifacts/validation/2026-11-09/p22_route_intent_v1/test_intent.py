import math
from intent import BranchIntent

HIT={'e1':[.65,-.5],'e2':[1.25,-.5]}
def setup():
    a=BranchIntent();a.frame([20],1.,1.02,0)
    assert not a.scan(1.10,1.0,[],None,0)
    return a

def test_two_motion_matched_scans_only_and_one_shot():
    a=setup()
    for i in range(1,4):
        assert not a.scan(1.1+i*.1,1+i*.1,[HIT],(0,0,math.radians(21)),0)
    # 같은 원 스캔·반복 영상은 새로운 증거가 아니다.
    a.frame([20],1.,1.41,0)
    assert not a.scan(1.42,1.3,[HIT],(0,0,0),0)
    moved={k:[v[0]-.15,v[1]] for k,v in HIT.items()}
    assert a.scan(1.50,1.40,[moved],(.15,0,0),0)
    a.frame([20],1.51,1.52,0)
    assert a.state=='spent'
    assert not a.scan(1.60,1.5,[moved],(0,0,0),0)

def test_moving_mismatch_and_missed_entry_do_not_select_exit():
    a=setup()
    for i in range(1,4):
        assert not a.scan(1.1+i*.1,1+i*.1,[HIT],(0,0,math.radians(22)),0)
    wrong={k:[v[0]+.25,v[1]] for k,v in HIT.items()}
    assert not a.scan(1.5,1.4,[wrong],(0,0,0),0)
    assert not a.scan(1.6,1.5,[],(0,0,-.70),0)
    assert a.reason=='wrong_or_next_turn'
    for i in range(3):
        assert not a.scan(1.7+i*.1,1.6+i*.1,[HIT],(0,0,0),0)

def test_vision_motion_or_reset_cannot_reuse_old_intent():
    a=BranchIntent();a.frame([20],1,1.55,0);assert a.state=='idle'
    a=setup();assert not a.scan(1.2,1.1,[HIT],None,0)
    assert a.reason=='motion_gap'
    a.reset(2.,1)
    a.frame([20],1.99,2.02,1);a.frame([20],2.01,2.02,0);assert a.state=='idle'
    a.frame([20],2.01,2.02,1)
    assert not a.scan(2.05,1.99,[HIT],None,1) and a.last_stamp is None
    a.frame([30],2.03,2.05,1);assert a.state=='idle'
