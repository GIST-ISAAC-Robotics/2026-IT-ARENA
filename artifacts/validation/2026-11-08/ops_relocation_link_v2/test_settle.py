from settle import hold_window
import math
def rows(v=0.,n=62):
    return [(i*.01,i*.01*v,0.) for i in range(n)]
def test_stationary_and_short_coverage():
    assert hold_window(rows())['passed']
    assert hold_window(rows(n=49)) is None
def test_reversal_is_not_a_stationary_net_displacement():
    r=rows();r[30]=(r[30][0],.003,0.)
    assert not hold_window(r)['passed']
def test_mean_can_hide_peak():
    r=rows(.01);r[40]=(r[40][0],.006,0.)
    assert not hold_window(r)['passed']
def test_gap_and_bad_values_do_not_measure_rest():
    r=rows();assert hold_window(r[:25]+r[40:]) is None
    r[27]=(r[27][0],math.nan,0.);assert hold_window(r) is None
def test_old_motion_not_counted_in_later_valid_window():
    r=rows(n=100);r[10]=(r[10][0],.03,0.)
    assert hold_window(r)['passed']
