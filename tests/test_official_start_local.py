import copy

import numpy as np
import pytest

from arena_autonomy.race_start import OfficialStartSignal
from arena_autonomy.race_start_local import LocalOfficialStartSignal, bank_window
from arena_autonomy.vision_process import make_start_signal, recognize
from audit_start_signal_layout import light_image


def image(red=0, green=False, clutter=True, **kwargs):
    rgb = light_image(red, green, **kwargs)
    if clutter:
        rgb[230:270, 130:500] = (255, 0, 0)
    return rgb


def arm(signal, **kwargs):
    for i in range(2):
        assert not signal.update(image(4, **kwargs), i*.1)
    assert signal.observed == 'red4_confirmed'


@pytest.mark.parametrize('diameter', [8, 12, 20, 28])
@pytest.mark.parametrize('paired', [False, True])
def test_red_vehicle_outside_array_does_not_block_valid_sequence(diameter, paired):
    signal = LocalOfficialStartSignal()
    arm(signal, diameter=diameter, paired=paired)
    for i in range(3):
        assert signal.update(image(0, True, diameter=diameter, paired=paired), .2+i*.1) == (i == 2)


def test_original_mode_still_rejects_global_unclassified_red():
    signal = OfficialStartSignal()
    for i in range(2):
        assert not signal.update(image(4), i*.1)
    assert signal.observed == 'red_unclassified'
    assert signal.bank is None


@pytest.mark.parametrize('kind', ['green_only', 'partial_red', 'no_green', 'unrelated_green'])
def test_no_signal_or_unprepared_green_does_not_start(kind):
    signal = LocalOfficialStartSignal()
    if kind in ('no_green', 'unrelated_green'):
        arm(signal)
    for i in range(12):
        assert not signal.update(image(2 if kind == 'partial_red' else 0, kind != 'no_green',
                                      green_shift=80 if kind == 'unrelated_green' else 0), .2+i*.1)


@pytest.mark.parametrize('paired', [False, True])
@pytest.mark.parametrize('red_count', [1, 3, 4])
def test_remaining_lamp_and_green_cannot_start(paired, red_count):
    signal = LocalOfficialStartSignal()
    arm(signal, paired=paired)
    for i in range(4):
        assert not signal.update(image(red_count, True, paired=paired), .2+i*.1)
    assert not signal.started


@pytest.mark.parametrize('phase', ['before', 'after'])
def test_malformed_red_touching_array_window_revokes_evidence(phase):
    signal = LocalOfficialStartSignal()
    if phase == 'after':
        arm(signal)
    rgb = image(4 if phase == 'before' else 0, True)
    # 중심은 창 밖이지만 긴 빨간 형상이 마지막 열/초록 예상 영역을 침범한다.
    rgb[112:116, 250:500] = (255, 0, 0)
    assert not signal.update(rgb, .2)
    assert signal.observed == 'red_unclassified'
    assert signal.bank is None


def test_initial_extra_lamp_like_red_is_ambiguous():
    signal = LocalOfficialStartSignal()
    rgb = image(4)
    rgb[90:102, 600:612] = (255, 0, 0)
    for i in range(3):
        assert not signal.update(rgb, i*.1)
    assert signal.bank is None


def test_extra_lamp_like_red_after_arming_is_still_ambiguous():
    signal = LocalOfficialStartSignal()
    arm(signal)
    rgb = image(0, True)
    rgb[90:102, 600:612] = (255, 0, 0)
    assert not signal.update(rgb, .2)
    assert signal.bank is None and signal.observed == 'red_partial_or_ambiguous'


def test_known_extinction_does_not_extend_transition_time():
    signal = LocalOfficialStartSignal()
    arm(signal)
    for i in range(10):
        assert not signal.update(image(3), .2+i*.1)
    assert signal.bank is None
    for i in range(3):
        assert not signal.update(image(0, True), 1.2+i*.1)


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), -1., True])
def test_bad_stamp_resets_even_with_recognizable_bank(stamp):
    signal = LocalOfficialStartSignal()
    arm(signal)
    assert not signal.update(image(0, True), stamp)
    assert signal.bank is None


def test_duplicate_and_gap_cannot_confirm():
    signal = LocalOfficialStartSignal()
    arm(signal)
    for _ in range(6):
        assert not signal.update(image(0, True), .2)
    assert signal.green_frames == 1
    assert not signal.update(image(0, True), .56)
    assert signal.bank is None


def test_recognition_worker_preserves_candidate_and_parent_state():
    mode = 'official_4red_local_candidate'
    parent = make_start_signal(mode)
    for i, rgb in enumerate([image(4)]*2+[image(0, True)]*3):
        before = copy.deepcopy(vars(parent))
        request = dict(epoch=0, sequence=i, stamp=i*.1, signal_mode=mode,
                       signal=before, image=dict(height=480, width=848, encoding='rgb8',
                       step=848*3, is_bigendian=False))
        result = recognize(request, rgb.tobytes())
        assert vars(parent) == before  # 처리만 끝난 결과는 부모의 출발 허가가 아니다.
        parent.__dict__.update(result['signal'])
        assert result['signal_mode'] == mode
    assert parent.started


def test_window_scales_from_observed_pixels():
    signal = LocalOfficialStartSignal()
    arm(signal, paired=True, diameter=12)
    left, top, right, bottom = bank_window(signal.bank)
    assert left < 170 and right > 170+4*24
    assert top < 125 and bottom > 141


def test_invalid_image_clears_local_evidence():
    signal = LocalOfficialStartSignal()
    arm(signal)
    assert not signal.update(np.zeros((3, 3), dtype=np.uint8), .2)
    assert signal.bank is None
