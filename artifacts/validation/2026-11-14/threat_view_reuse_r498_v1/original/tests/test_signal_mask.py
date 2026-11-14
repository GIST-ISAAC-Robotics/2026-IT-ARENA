"""신호 영상 연산 최적화는 색상 경계·판정 결과를 바꾸지 않는다."""
import itertools
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_autonomy'))
from arena_autonomy import core


def legacy_mask(hsv):
    return (((hsv[:, :, 0] < 12) | (hsv[:, :, 0] > 170)) &
            (hsv[:, :, 1] > 140) & (hsv[:, :, 2] > 160)).astype(np.uint8)*255


def test_mask_matches_random_pixels_and_all_threshold_boundaries():
    hsv = np.random.default_rng(20260928).integers(0, 256, (500, 600, 3), dtype=np.uint8)
    assert np.array_equal(core.red_signal_mask(hsv), legacy_mask(hsv))
    boundary = np.array(list(itertools.product(
        [0, 11, 12, 170, 171, 179, 255], [0, 139, 140, 141, 255],
        [0, 159, 160, 161, 255])), dtype=np.uint8).reshape(1, -1, 3)
    assert np.array_equal(core.red_signal_mask(boundary), legacy_mask(boundary))
    # 잘린 ROI처럼 연속적이지 않은 배열도 같은 결과여야 한다.
    assert np.array_equal(core.red_signal_mask(hsv[::2, ::2]), legacy_mask(hsv[::2, ::2]))


def test_signal_state_sequence_exactly_matches_old_mask(monkeypatch):
    def frame(state):
        rgb = np.full((480, 848, 3), 100, np.uint8)
        cv2.rectangle(rgb, (250, 130), (390, 180), (3, 3, 3), -1)
        for name, x, color in [('red', 280, (255, 4, 1)), ('yellow', 320, (255, 180, 1)),
                                ('green', 360, (1, 255, 10))]:
            cv2.circle(rgb, (x, 155), 10, color if state == name else (0, 0, 0), -1)
        return rgb
    frames = [frame(s) for s in ('green', 'red', 'red', 'yellow', 'green', 'yellow',
                                 'green', 'green', 'green', 'red')]
    optimized = core.StartSignal()
    expected = []
    for rgb in frames:
        optimized.update(rgb)
        expected.append(vars(optimized).copy())
    assert expected[-1]['started']
    monkeypatch.setattr(core, 'red_signal_mask', legacy_mask)
    previous = core.StartSignal()
    actual = []
    for rgb in frames:
        previous.update(rgb)
        actual.append(vars(previous).copy())
    assert actual == expected
