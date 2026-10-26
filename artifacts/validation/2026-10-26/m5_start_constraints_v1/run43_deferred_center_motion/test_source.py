"""출발 입력의 거리 편향은 무반환/근접 무효를 복구하거나 취득 시각을 바꾸지 않는다."""
import math
import numpy as np
import pytest
from simulate_grid_departure import range_bias, event_key, peer_active_time
from audit_grid_departure_motion import audited_peer_time


@pytest.mark.parametrize('bias',[-.03,0.,.03])
def test_invalid_states_and_original_input_are_preserved(bias):
    original=np.array([np.nan,np.inf,-np.inf,.05,.08,.5,11.99,12.])
    saved=original.copy();out=range_bias(original,bias,.05,12.)
    np.testing.assert_array_equal(original,saved)
    assert np.isnan(out[0]) and out[1]==np.inf and out[2]==-np.inf
    assert out[5]==.5+bias
    if bias<0:assert np.isnan(out[3])
    else:assert out[3]==.05+bias
    if bias>0:assert out[6]==np.inf and out[7]==np.inf
    if bias==0:np.testing.assert_array_equal(out,original)


@pytest.mark.parametrize('bad',[True,math.nan,math.inf,-math.inf])
def test_invalid_bias_rejected(bad):
    with pytest.raises(ValueError):range_bias([.5],bad,.05,12.)


def test_extra_delivery_delay_changes_receipt_only():
    acquired=.5;payload={'first_s':acquired}
    delivered=acquired+.0998+.002+.05
    event=event_key(delivered,0,'scan_delivery',payload)
    assert event[0]==651800000 and event[-1]['first_s']==.5
    assert event<event_key(delivered,1,'control',None)


@pytest.mark.parametrize('now,expected',[(0.,0.),(.5,0.),(1.,.5),(1.8,1.3),(3.,1.3),(3.8,1.3),(4.,1.5),(6.,3.5)])
def test_pause_acquisition_and_independent_audit_match_registered_motion(now,expected):
    assert peer_active_time(now,1.8,3.8)==pytest.approx(expected)
    assert audited_peer_time(now,dict(peer_stop_s=1.8,peer_resume_s=3.8))==pytest.approx(expected)
    assert peer_active_time(now)==max(0.,now-.5)


@pytest.mark.parametrize('stop,resume',[(None,3.8),(1.8,None),(.2,3.8),(3.8,1.8),(math.nan,3.8),(1.8,math.inf)])
def test_invalid_actor_profile_is_not_silently_changed(stop,resume):
    with pytest.raises(ValueError):peer_active_time(2.,stop,resume)
    with pytest.raises(ValueError):audited_peer_time(2.,dict(peer_stop_s=stop,peer_resume_s=resume))
