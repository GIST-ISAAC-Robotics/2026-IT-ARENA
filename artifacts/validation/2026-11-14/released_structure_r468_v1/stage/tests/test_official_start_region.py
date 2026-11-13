"""출발 판독 영역: 관측 배열만 근거로 넓히고 영상 밖·불완전 광원은 판독으로 치지 않는다."""
import numpy as np
import pytest

from arena_autonomy.race_start import OfficialStartSignal, _lamps_inside
from arena_autonomy.race_start_local import LocalOfficialStartSignal
from audit_start_signal_layout import light_image
import cv2

DETECTORS = (OfficialStartSignal, LocalOfficialStartSignal)
LOW = dict(offset=(170, 340), diameter=12, paired=True)  # 480 px 영상의 58% 선(278) 아래


def feed(signal, frames, start=0):
    for i, frame in enumerate(frames):
        signal.update(frame, (start+i)*.1)
    return signal


@pytest.mark.parametrize('detector', DETECTORS)
def test_array_below_old_top_region_starts_only_after_full_sequence(detector):
    probe = light_image(green=True, **LOW)
    assert not feed(detector(), [light_image(**LOW)]*3+[probe]*5).started
    signal = feed(detector(), [probe]*3+[light_image(reds=4, **LOW)]*3+[light_image(green=True, **LOW)]*3)
    assert signal.started and signal.observed == 'green_confirmed'


@pytest.mark.parametrize('detector', DETECTORS)
def test_low_partial_extinction_is_not_read_as_lights_out(detector):
    # 예전 상단 영역으로 돌아가면 남은 빨강이 사라진 것처럼 보이는 경우.
    red = light_image(reds=4, **LOW)
    remaining = light_image(reds=2, green=True, **LOW)
    signal = feed(detector(), [red]*3+[remaining]*6)
    assert not signal.started and signal.green_frames == 0


@pytest.mark.parametrize('detector', DETECTORS)
def test_extra_low_lamp_blocks_widening_and_never_picks_a_subset(detector):
    red = light_image(reds=4, **LOW)
    cv2.circle(red, (700, 420), 6, (255, 4, 1), -1)
    green = light_image(green=True, **LOW)
    signal = feed(detector(), [red]*3+[green]*4)
    assert not signal.started and signal.bank is None and signal.observed == 'waiting_red4'


@pytest.mark.parametrize('detector', DETECTORS)
def test_top_bank_keeps_old_reading_despite_red_below(detector):
    # 상단 배열과 도로 쪽 빨강이 함께면 전체 해석은 실패하고 기존 상단 판독을 쓴다.
    kwargs = dict(offset=(170, 125), diameter=12, paired=True)
    red = light_image(reds=4, **kwargs)
    green = light_image(green=True, **kwargs)
    cv2.circle(red, (700, 420), 6, (255, 4, 1), -1)
    cv2.circle(green, (700, 420), 6, (255, 4, 1), -1)
    assert feed(detector(), [red]*3+[green]*3).started


@pytest.mark.parametrize('detector', DETECTORS)
def test_green_elsewhere_or_shifted_lamp_does_not_start_low_array(detector):
    red = light_image(reds=4, **LOW)
    moved = light_image(green=True, green_shift=40, **LOW)
    assert not feed(detector(), [red]*3+[moved]*5).started
    other = np.full_like(red, 100)
    cv2.circle(other, (120, 120), 6, (1, 255, 10), -1)
    cv2.circle(other, (120, 136), 6, (1, 255, 10), -1)
    assert not feed(detector(), [red]*3+[other]*5).started


@pytest.mark.parametrize('detector', DETECTORS)
def test_one_of_paired_green_lamps_is_not_enough(detector):
    red = light_image(reds=4, **LOW)
    green = light_image(green=True, **LOW)
    bottom = round(340+16)
    x = round(170+4*24)
    green[bottom-9:bottom+10, x-9:x+10] = 100
    signal = feed(detector(), [red]*3+[green]*5)
    assert not signal.started


@pytest.mark.parametrize('detector', DETECTORS)
def test_search_margin_may_be_clipped_when_the_lamps_themselves_are_visible(detector):
    kwargs = dict(offset=(170, 8), diameter=12, paired=True)  # 예전 사각형은 영상 위로 나갔다.
    assert feed(detector(), [light_image(reds=4, **kwargs)]*3+[light_image(green=True, **kwargs)]*3).started


def test_clipped_lamp_is_not_inside_even_if_its_center_is():
    bank = dict(gx=100., gy=4., vertical=0., diameter=12., pairs=1)
    assert not _lamps_inside(bank, (480, 848, 3))
    assert not _lamps_inside(dict(bank, gy=30., gx=844.), (480, 848, 3))
    assert not _lamps_inside(dict(bank, gy=470., vertical=8., pairs=2), (480, 848, 3))
    assert _lamps_inside(dict(bank, gy=30.), (480, 848, 3))


@pytest.mark.parametrize('detector', DETECTORS)
@pytest.mark.parametrize('offset', [(170, 3), (170, 476)])
def test_green_clipped_by_image_edge_cannot_start(detector, offset):
    kwargs = dict(offset=offset, diameter=12, paired=False)
    signal = feed(detector(), [light_image(reds=4, **kwargs)]*3+[light_image(green=True, **kwargs)]*5)
    assert not signal.started


def test_low_array_with_red_body_elsewhere_uses_local_rule_only():
    # 광원 모양이 아닌 빨간 차체는 지역 검출기에서만 배열 창 밖으로 분리된다.
    red = light_image(reds=4, **LOW)
    green = light_image(green=True, **LOW)
    for frame in (red, green):
        cv2.rectangle(frame, (500, 30), (740, 90), (255, 4, 1), -1)
    assert feed(LocalOfficialStartSignal(), [red]*3+[green]*3).started
    assert not feed(OfficialStartSignal(), [red]*3+[green]*3).started
