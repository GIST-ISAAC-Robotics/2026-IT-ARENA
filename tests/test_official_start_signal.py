import math

import numpy as np
import pytest

from arena_autonomy.race_start import OfficialStartSignal, StartConfig
from audit_start_signal_layout import light_image


def reds(detector, diameter=12, paired=False, start=0., **kwargs):
    for i, n in enumerate([0, 1, 2, 3, 4, 4]):
        assert not detector.update(light_image(n, diameter=diameter, paired=paired, **kwargs), start+i*.1)
    return start+.6


@pytest.mark.parametrize('diameter', [8, 12, 20, 28])
@pytest.mark.parametrize('paired', [False, True])
def test_observed_four_reds_then_green_latches(diameter, paired):
    signal = OfficialStartSignal()
    t = reds(signal, diameter, paired)
    image = light_image(0, True, diameter, paired=paired)
    assert not signal.update(image, t)
    assert not signal.update(image, t+.1)
    assert signal.update(image, t+.2)
    assert signal.update(light_image(), t+5.)  # 초록 소등 뒤에도 경기 출발 허가는 유지


def test_five_pixel_circle_has_insufficient_contour_area():
    signal = OfficialStartSignal()
    t = reds(signal, diameter=5)
    for i in range(4):
        assert not signal.update(light_image(0, True, diameter=5), t+i*.1)


@pytest.mark.parametrize('kind', ['green_only', 'dark_only', 'partial', 'red_and_green', 'unrelated_green', 'grass'])
def test_ambiguous_or_unprepared_start_is_rejected(kind):
    signal = OfficialStartSignal()
    if kind in ('green_only', 'dark_only'):
        t = 0.
    else:
        t = reds(signal)
    for i in range(12):
        image = light_image(reds=2 if kind == 'partial' else (4 if kind == 'red_and_green' else 0),
                            green=kind not in ('dark_only', 'partial', 'grass'),
                            green_shift=70 if kind == 'unrelated_green' else 0)
        if kind == 'grass':
            image[280:] = (0, 255, 0)
        assert not signal.update(image, t+i*.1)


def test_duplicate_frames_do_not_confirm_and_backward_time_revokes_start():
    signal = OfficialStartSignal()
    t = reds(signal)
    image = light_image(0, True)
    for _ in range(10):
        assert not signal.update(image, t)
    assert signal.green_frames == 1
    assert not signal.update(image, t+.1)
    assert signal.update(image, t+.2)
    assert not signal.update(image, t-.1)
    assert not signal.started and signal.bank is None


@pytest.mark.parametrize('gap', [.36, 1., 10.])
def test_frame_gap_cannot_reuse_old_red(gap):
    signal = OfficialStartSignal()
    reds(signal)
    for i in range(4):
        assert not signal.update(light_image(0, True), .5+gap+i*.1)


def test_known_red_extinction_keeps_pattern_but_requires_green():
    signal = OfficialStartSignal()
    t = reds(signal)
    assert not signal.update(light_image(3), t)
    assert signal.observed == 'red_extinction_pending'
    assert signal.last_red == pytest.approx(t-.1)
    assert not signal.update(light_image(0, True), t+.1)
    assert not signal.update(light_image(0, True), t+.2)
    assert signal.update(light_image(0, True), t+.3)


@pytest.mark.parametrize('paired', [False, True])
def test_red_subset_and_green_simultaneous_never_starts(paired):
    signal = OfficialStartSignal()
    t = reds(signal, paired=paired)
    for i, count in enumerate((3, 2, 1)):
        assert not signal.update(light_image(count, True, paired=paired), t+i*.1)
        assert signal.green_frames == 0
    for i in range(3):
        started = signal.update(light_image(0, True, paired=paired), t+.3+i*.1)
        assert started == (i == 2)


def test_partial_red_never_renews_last_full_pattern_deadline():
    signal = OfficialStartSignal()
    t = reds(signal)
    for i in range(10):
        assert not signal.update(light_image(3), t+i*.1)
    assert signal.bank is None
    for i in range(3):
        assert not signal.update(light_image(0, True), t+1.+i*.1)


@pytest.mark.parametrize('change', ['shift', 'scale', 'additional'])
def test_unknown_red_subset_revokes_pattern(change):
    signal = OfficialStartSignal()
    t = reds(signal)
    frame = light_image(3, offset=(182,125)) if change == 'shift' else light_image(3)
    if change == 'scale':
        frame = light_image(3, red_scale=[2.5]*5)
    elif change == 'additional':
        frame[90:105,400:415] = (255,0,0)
    assert not signal.update(frame, t)
    assert signal.bank is None
    for i in range(3):
        assert not signal.update(light_image(0,True),t+.1+i*.1)


@pytest.mark.parametrize('red_count', [0, 2, 4])
def test_unclassified_red_is_not_extinction_or_confirmed_bank(red_count):
    signal = OfficialStartSignal()
    t = reds(signal)
    frame = light_image(red_count, green=True)
    frame[90:100,400:480] = (255,0,0)  # 면적은 크지만 광원 모양이 아닌 빨강
    assert not signal.update(frame,t)
    assert signal.observed == 'red_unclassified'
    assert signal.bank is None
    for i in range(3):
        assert not signal.update(light_image(0,True),t+.1+i*.1)


def test_small_red_pixel_noise_does_not_count_as_a_signal():
    signal = OfficialStartSignal()
    t = reds(signal)
    frame = light_image(0,True)
    frame[90:92,400:402] = (255,0,0)
    for i in range(3):
        assert signal.update(frame,t+i*.1) == (i == 2)


def test_incomplete_pattern_was_never_armed():
    signal = OfficialStartSignal()
    for i in range(3):
        assert not signal.update(light_image(3), i*.1)
    for i in range(3):
        assert not signal.update(light_image(0,True), .3+i*.1)


def test_frame_gap_after_known_extinction_still_discards_pattern():
    signal = OfficialStartSignal()
    t = reds(signal)
    assert not signal.update(light_image(3), t)
    for i in range(3):
        assert not signal.update(light_image(0,True),t+.36+i*.1)


def test_green_outside_image_cannot_start():
    signal = OfficialStartSignal()
    t = reds(signal, offset=(760, 125))
    for i in range(4):
        assert not signal.update(light_image(0, True, offset=(760, 125)), t+i*.1)


@pytest.mark.parametrize('stamp', [math.nan, math.inf, -1., True])
def test_invalid_stamp_resets(stamp):
    signal = OfficialStartSignal()
    reds(signal)
    assert not signal.update(light_image(0, True), stamp)
    assert signal.bank is None and not signal.started


def test_invalid_image_discards_partial_evidence():
    signal = OfficialStartSignal()
    t = reds(signal)
    assert not signal.update(np.zeros((5, 5)), t)
    assert signal.bank is None


@pytest.mark.parametrize('kwargs', [{'red_confirmations': 1}, {'green_confirmations': True},
                                 {'max_frame_gap_s': math.nan}, {'max_red_to_green_s': .1}])
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        StartConfig(**kwargs)
