"""원시 두 스캔 진단 후보. 일치/불일치 판정은 안전 또는 실제 정지 증명이 아니다."""
import math

import numpy as np
import pytest

from scan_motion_diagnostic import Settings, compare_scans, prepare
from validate_scan_motion_diagnostic import encoder_travel, count_series, synthetic_scan


@pytest.mark.parametrize('bad', [0., -1., math.nan])
def test_invalid_settings(bad):
    with pytest.raises(ValueError):
        Settings(correspondence_m=bad)


@pytest.mark.parametrize('key,value', [('time_increment', 0.), ('angle_increment', -.01), ('stamp_s', -1.)])
def test_invalid_metadata_is_unknown(key, value):
    a, b = synthetic_scan(.2), synthetic_scan(.5)
    b[key] = value
    assert compare_scans(a, b, 0.)['classification'] == 'unknown'


def test_missing_encoder_is_not_zero():
    result = compare_scans(synthetic_scan(.2), synthetic_scan(.5), None)
    assert result['classification'] == 'unknown' and result['reason'] == 'encoder_history_missing'


def test_old_pair_not_allowed():
    result = compare_scans(synthetic_scan(.2), synthetic_scan(.7), 0.)
    assert result['reason'] == 'pair_time_gap'


def test_non_returns_not_filled_or_connected():
    scan = synthetic_scan(.2)
    scan['ranges'] = [math.inf]*500
    with pytest.raises(ValueError, match='insufficient_returns'):
        prepare(scan, Settings())


def test_counters_use_motor_ratio_and_capture_time():
    rows = [{'feedback_capture_us': i*10_000, 'motor_counts': i*10, 'feedback_valid': True,
             'feedback_mode': 'drive_motor_shaft'} for i in range(5)]
    t, c = count_series(rows+rows)
    assert len(t) == 5
    assert encoder_travel(t, c, .005, .025) == pytest.approx(20*math.tau*.0325/420)
    assert encoder_travel(t, c, -.01, .02) is None
    assert encoder_travel(t, c, .03, .05) is None
    assert encoder_travel(np.array([0., .1]), np.array([0., 100.]), .02, .04) is None


def test_conflicting_counter_record_rejected():
    rows = [{'feedback_capture_us': 10, 'motor_counts': v, 'feedback_valid': True,
             'feedback_mode': 'drive_motor_shaft'} for v in (1, 2)]
    with pytest.raises(ValueError, match='conflicting'):
        count_series(rows)


def test_never_confirms_stationary_or_static_scene():
    result = compare_scans(synthetic_scan(.2), synthetic_scan(.5), 0.)
    assert result['stationary_confirmed'] is False
    assert result['static_scene_assumption_verified'] is False


def test_stationary_room_has_multistart_agreement_without_encoder_prior():
    a, b = synthetic_scan(.2), synthetic_scan(.5)
    zero = compare_scans(a, b, 0.)
    wrong_encoder = compare_scans(a, b, .45)
    assert zero['classification'] == 'compatible_under_assumptions'
    assert wrong_encoder['classification'] == 'inconsistent_candidate'
    assert zero['forward']['transform'] == wrong_encoder['forward']['transform']
