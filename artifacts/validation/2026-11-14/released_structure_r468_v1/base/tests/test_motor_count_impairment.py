import math
import pytest

from arena_vehicle_interface.motor_count_impairment import MotorCountTransform
from arena_vehicle_interface.motion_impairment import StreamFault


@pytest.mark.parametrize('scale',[.95,1.,1.05])
@pytest.mark.parametrize('direction',[-1,1])
def test_signed_counts_and_velocity_share_one_quantized_history(scale,direction):
    tr=MotorCountTransform(28); spec=tr.spec; previous=None
    for i in range(101):
        t=i*.01; raw=100+direction*i
        angle,v=tr.apply(spec.motor_angle_from_counts(raw), math.nan if i==0 else direction*spec.motor_rad_per_count/.01,t,StreamFault(scale=scale))
        count=spec.counts_from_position(angle)
        assert abs(count-(100+scale*direction*i)) <= .500000001
        if i==0:assert math.isnan(v)
        else:assert v==pytest.approx((count-previous)*spec.motor_rad_per_count/.01)
        previous=count


def test_gain_window_does_not_reset_accumulated_error_or_jump_position():
    tr=MotorCountTransform(28); s=tr.spec
    fault=StreamFault(scale=.5,start_s=.015,end_s=.035)
    result=[]
    for i in range(6):
        pos,_=tr.apply(s.motor_angle_from_counts(i*10),1.,i*.01,fault)
        result.append(s.counts_from_position(pos))
    assert result==[0,10,18,22,30,40]


def test_invalid_position_time_and_reset_preserve_input_contract():
    tr=MotorCountTransform(28); s=tr.spec
    with pytest.raises(ValueError):tr.apply(.95*s.motor_rad_per_count,1.,0.,StreamFault())
    tr.apply(0.,math.nan,1.,StreamFault())
    with pytest.raises(ValueError):tr.apply(0.,0.,1.,StreamFault())
    with pytest.raises(ValueError):tr.apply(0.,0.,.9,StreamFault())
    tr.reset()
    angle,v=tr.apply(s.motor_angle_from_counts(-3),math.nan,.1,StreamFault(scale=.95))
    assert s.counts_from_position(angle)==-3 and math.isnan(v)


def test_nonfinite_later_velocity_is_not_repaired():
    tr=MotorCountTransform(28)
    tr.apply(0.,math.nan,0.,StreamFault())
    _,v=tr.apply(0.,math.nan,.01,StreamFault())
    assert math.isnan(v)
