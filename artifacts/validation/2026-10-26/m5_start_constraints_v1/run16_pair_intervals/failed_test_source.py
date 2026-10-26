import numpy as np
import pytest

from paired_wall_interval_probe import center_interval, binned_pair_probe
from arena_autonomy.lateral_intent import LateralIntentConfig, PathFrame


@pytest.mark.parametrize('e_left,e_right', [(.029,.029), (-.029,-.029), (.029,-.029), (-.029,.029), (0.,0.)])
def test_independent_signed_errors_keep_true_center(e_left, e_right):
    # radial error shifts right-wall signed distance in the opposite direction.
    center = .11
    pair = center_interval([center+.425+e_left], [center-.425-e_right], error_m=.03)
    assert pair is not None and pair[0] <= center <= pair[1]


def test_closed_interval_boundary_without_decimal_rounding_ambiguity():
    assert center_interval([.65625], [-.40625], wall_half_m=.5, error_m=.03125) == (.125,.125)


def test_means_cannot_hide_inconsistent_measurement_or_width():
    assert center_interval([.425,.51], [-.425]) is None
    assert center_interval([.475], [-.475]) is None
    assert center_interval([], [-.425]) is None
    assert center_interval([.425], []) is None
    interval = center_interval([.435], [-.415])
    assert interval == pytest.approx((-.02,.04))


def test_no_interpolation_into_missing_sections_and_no_runtime_authorization():
    frame = PathFrame(np.array([[0.,0.],[1.,0.]]))
    cfg = LateralIntentConfig()
    pts = np.array([[.01,.425],[.01,-.425],[.11,.425],[.11,-.425]])
    result = binned_pair_probe(frame, pts, 0., .15, cfg)
    assert result['compatible_bins'] == 2 and result['missing_bins'] == 1
    assert result['driving_authorized'] is False


@pytest.mark.parametrize('left,right', [([np.nan],[-.425]), ([[.425]],[-.425]), ([.425],[np.inf])])
def test_invalid_input_is_not_free_space(left, right):
    with pytest.raises(ValueError): center_interval(left, right)
