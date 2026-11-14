import numpy as np

from validate_official_start import image, run_case
from arena_autonomy.race_start import StartConfig


def test_fixture_is_rgb_and_not_a_detector_state_channel():
    rgb = image(4, False, width=640, height=360)
    assert rgb.shape == (360, 640, 3) and rgb.dtype == np.uint8
    assert np.count_nonzero(rgb[:, :, 0] > 200) > 0


def test_expectation_and_exploratory_failure_are_distinct():
    args = dict(config=StartConfig(), camera=dict(width=640, height=360, fov=75.), distance=10.)
    assert run_case('too_small', expected='start', **args)['passed'] is False
    assert run_case('too_small', expected='no_start', **args)['passed'] is True
    assert run_case('too_small', expected='exploratory', **args)['passed'] is None


def test_clean_near_sequence_requires_visual_green_and_reports_latency():
    result = run_case('clean', expected='start', config=StartConfig(),
                      camera=dict(width=640, height=360, fov=75.), distance=.5)
    assert result['passed'] and not result['false_start']
    assert .19 < result['detection_delay_s'] < .21
