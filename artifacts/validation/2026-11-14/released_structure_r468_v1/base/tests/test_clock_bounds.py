from dataclasses import replace
from fractions import Fraction

import pytest

from arena_vehicle_interface.clock_bounds import ClockAssumptions,ClockExchange,fit_clock


def assumptions(**kw):
    values=dict(max_rate_error_ppm=200,host_timestamp_error_ns=0,mcu_timestamp_error_us=0,
                max_window_us=5_000_000,max_extrapolation_us=100_000)
    values.update(kw)
    return ClockAssumptions(**values)


def exchange(seq=0,mcu=1_000_000,forward_ns=1_000_000,reverse_ns=9_000_000,rate=Fraction(1),offset=8_000_000_000):
    processing=100
    # 합성 정답은 Fraction으로 계산하며 시험에서는 ns 정수에 맞는 시각을 선택한다.
    arrival=rate*mcu*1000+offset
    departure=rate*(mcu+processing)*1000+offset
    assert arrival.denominator==departure.denominator==1
    return ClockExchange('boot-A',seq,int(arrival)-forward_ns,mcu,mcu+processing,int(departure)+reverse_ns)


def test_asymmetric_delay_is_interval_not_half_rtt_point():
    record=exchange()
    result=fit_clock([record],assumptions(max_rate_error_ppm=0)).map_interval(record.mcu_receive_us,boot='boot-A')
    assert result.host_lower_ns==8_999_000_000
    assert result.host_upper_ns==9_009_000_000
    assert result.width_ns==10_000_000
    truth=9_000_000_000
    assert result.host_lower_ns<=truth<=result.host_upper_ns
    assert (result.host_lower_ns+result.host_upper_ns)//2!=truth
    assert not result.clock_mapping_applied and not result.ros_stamp_verified and not result.ready_to_drive


def test_multiple_exchanges_intersect_without_assuming_equal_delay():
    records=[exchange(),exchange(1,2_000_000,9_000_000,1_000_000)]
    fit=fit_clock(records,assumptions(max_rate_error_ppm=0))
    result=fit.map_interval(1_500_000,boot='boot-A')
    assert result.width_ns==2_000_000
    assert result.host_lower_ns<=9_500_000_000<=result.host_upper_ns


@pytest.mark.parametrize('ppm',[-150,0,150])
def test_exact_known_rate_inside_bounds_is_contained(ppm):
    rate=Fraction(1_000_000+ppm,1_000_000)
    records=[exchange(i,1_000_000+i*500_000,500_000+i*100_000,1_000_000,rate) for i in range(6)]
    fit=fit_clock(records,assumptions())
    for stamp in (1_000_000,1_250_000,3_000_000,3_600_000):
        interval=fit.map_interval(stamp,boot='boot-A')
        truth=rate*stamp*1000+8_000_000_000
        assert interval.host_lower_ns<=truth<=interval.host_upper_ns


def test_integer_precision_beyond_float_exact_range():
    offset=2**60+123
    record=exchange(forward_ns=7,reverse_ns=11,offset=offset)
    result=fit_clock([record],assumptions(max_rate_error_ppm=0)).map_interval(1_000_000,boot='boot-A')
    assert result.host_lower_ns==offset+1_000_000_000-7
    assert result.host_upper_ns==offset+1_000_000_000+11


def test_zero_delay_and_rate_form_degenerate_feasible_point():
    record=exchange(forward_ns=0,reverse_ns=0)
    fit=fit_clock([record],assumptions(max_rate_error_ppm=0))
    result=fit.map_interval(1_000_001,boot='boot-A')
    assert result.width_ns==0 and result.host_lower_ns==9_000_001_000


def test_clock_conflict_rejected_not_least_squares_fit():
    records=[exchange(forward_ns=0,reverse_ns=0),exchange(1,2_000_000,0,0,offset=8_001_000_000)]
    with pytest.raises(ValueError,match='inconsistent'):
        fit_clock(records,assumptions(max_rate_error_ppm=0))


def test_timestamp_uncertainty_expands_mapping():
    record=exchange(forward_ns=0,reverse_ns=0)
    precise=fit_clock([record],assumptions(max_rate_error_ppm=0)).map_interval(1_000_000,boot='boot-A')
    uncertain=fit_clock([record],assumptions(max_rate_error_ppm=0,host_timestamp_error_ns=5,
        mcu_timestamp_error_us=1)).map_interval(1_000_000,boot='boot-A')
    assert uncertain.host_lower_ns==precise.host_lower_ns-2005
    assert uncertain.host_upper_ns==precise.host_upper_ns+2005


def test_extrapolation_and_boot_are_not_implicitly_extended():
    fit=fit_clock([exchange()],assumptions(max_extrapolation_us=10))
    assert fit.map_interval(1_000_110,boot='boot-A').extrapolation_us==10
    with pytest.raises(ValueError,match='extrapolation'): fit.map_interval(1_000_111,boot='boot-A')
    with pytest.raises(ValueError,match='boot'): fit.map_interval(1_000_000,boot='boot-B')


@pytest.mark.parametrize('fault',['boot','sequence','overlap','rewind','window','capacity'])
def test_invalid_fit_input(fault):
    records=[exchange(),exchange(1,2_000_000)]
    if fault=='boot': records[1]=replace(records[1],boot='boot-B')
    elif fault=='sequence': records[1]=replace(records[1],sequence=0)
    elif fault=='overlap': records[1]=replace(records[1],host_send_ns=records[0].host_receive_ns-1)
    elif fault=='rewind': records[1]=replace(records[1],mcu_receive_us=1,mcu_send_us=2)
    elif fault=='window': records[1]=exchange(1,7_000_000)
    elif fault=='capacity': records=[exchange()]*65
    with pytest.raises(ValueError): fit_clock(records,assumptions())


def test_sequence_gaps_reported_not_filled():
    fit=fit_clock([exchange(),exchange(3,2_000_000)],assumptions())
    assert fit.sequence_gaps==2


@pytest.mark.parametrize('kwargs',[{'max_rate_error_ppm':True},{'max_window_us':0},
    {'host_timestamp_error_ns':-1},{'mcu_timestamp_error_us':float('nan')},
    {'max_extrapolation_us':-1},{'max_rate_error_ppm':1_000_000}])
def test_invalid_assumptions(kwargs):
    with pytest.raises(ValueError): assumptions(**kwargs)


def test_invalid_exchange_values():
    with pytest.raises(ValueError): replace(exchange(),host_send_ns=True)
    with pytest.raises(ValueError): replace(exchange(),mcu_send_us=0)
    with pytest.raises(ValueError): replace(exchange(),boot='')
