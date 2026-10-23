"""B3 시험 후보: 공식 4빨강+1초록의 시각 출발 인식.

운영 launch 미연결. 픽셀·카메라 시각만 사용하며 공식 신호 상태/시간표는
입력하지 않는다. 설치/노출/실물 성능은 미검증이다.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from arena_autonomy.core import red_signal_mask


@dataclass(frozen=True)
class StartConfig:
    red_confirmations: int = 2
    green_confirmations: int = 3
    max_frame_gap_s: float = .35
    max_red_to_green_s: float = .8

    def __post_init__(self):
        for count in (self.red_confirmations, self.green_confirmations):
            if isinstance(count, bool) or not isinstance(count, int) or count < 2:
                raise ValueError('at least two distinct frames required')
        if (any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in
                (self.max_frame_gap_s, self.max_red_to_green_s)) or
                self.max_red_to_green_s < self.max_frame_gap_s):
            raise ValueError('invalid start timing contract')


def _blobs(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = []
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        if (5 <= width <= 100 and 4 <= height <= 100 and
                .55 < width/height < 1.8 and cv2.contourArea(contour) > 10):
            output.append((x+width/2, y+height/2, float(width)))
    return output


def _red_bank(blobs):
    """같은 x의 위아래 광원을 묶고 4열의 일정한 배열을 확인한다.

    별도의 빨강 물체까지 포함해 열이 4개를 초과하면 미확인이다. 도로/다른
    차량의 빨강과 항상 구별한다는 가정은 하지 않는다.
    """
    if not 4 <= len(blobs) <= 8:
        return None
    diameter = float(np.median([b[2] for b in blobs]))
    columns = []
    for blob in sorted(blobs):
        if columns and abs(blob[0]-np.mean([b[0] for b in columns[-1]])) <= diameter*.6:
            columns[-1].append(blob)
        else:
            columns.append([blob])
    if len(columns) != 4 or any(len(c) > 2 for c in columns):
        return None
    if len({len(c) for c in columns}) != 1:
        return None
    centers = np.asarray([[np.mean([b[k] for b in c]) for k in (0, 1)] for c in columns])
    diameters = np.asarray([b[2] for b in blobs])
    if np.max(diameters)/np.min(diameters) > 1.8:
        return None
    gaps = np.diff(centers[:, 0])
    pitch = float(np.mean(gaps))
    if not .8*diameter <= pitch <= 4*diameter or np.max(np.abs(gaps-pitch)) > .25*pitch:
        return None
    slope, intercept = np.polyfit(centers[:, 0], centers[:, 1], 1)
    if abs(slope) > .4 or np.max(np.abs(centers[:, 1]-(slope*centers[:, 0]+intercept))) > .35*diameter:
        return None
    pair_offsets = [b[1]-np.mean([p[1] for p in c]) for c in columns for b in c]
    vertical = float(max(abs(v) for v in pair_offsets))
    if vertical > 1.5*diameter:
        return None
    gx = float(centers[-1, 0]+pitch)
    return dict(cx=float(np.mean(centers[:, 0])), cy=float(np.mean(centers[:, 1])),
                diameter=diameter, pitch=pitch, gx=gx, gy=float(slope*gx+intercept),
                vertical=vertical, pairs=len(columns[0]))


class OfficialStartSignal:
    def __init__(self, config=None):
        self.config = config or StartConfig()
        self.reset()

    def reset(self):
        self.started = False
        self.observed = 'unknown'
        self.bank = None
        self.red_frames = self.green_frames = 0
        self.last_stamp = self.last_red = None

    def _forget_pattern(self):
        self.bank = None
        self.red_frames = self.green_frames = 0
        self.last_red = None

    def update(self, rgb, stamp_s):
        if isinstance(stamp_s, bool) or not isinstance(stamp_s, (float, int)) or not math.isfinite(stamp_s) or stamp_s < 0:
            self.reset()
            self.observed = 'invalid_stamp'
            return False
        if self.last_stamp is not None and stamp_s <= self.last_stamp:
            # 중복은 확인 수에 포함하지 않는다. 되감기는 새 경기 상태다.
            if stamp_s < self.last_stamp:
                self.reset()
                self.observed = 'clock_reset'
                return False
            return self.started
        if self.started:
            self.last_stamp = stamp_s
            return True
        if self.last_stamp is not None and stamp_s-self.last_stamp > self.config.max_frame_gap_s:
            self._forget_pattern()
        self.last_stamp = stamp_s
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 4:
            self._forget_pattern()
            self.observed = 'invalid_image'
            return False
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        red = red_signal_mask(hsv)
        red[int(rgb.shape[0]*.58):] = 0
        blobs = _blobs(red)
        bank = _red_bank(blobs)
        if bank is not None:
            consistent = (self.bank is not None and
                          math.hypot(bank['cx']-self.bank['cx'], bank['cy']-self.bank['cy']) <= max(3., .5*bank['diameter']) and
                          abs(bank['pitch']-self.bank['pitch']) <= .25*bank['pitch'] and
                          bank['pairs'] == self.bank['pairs'])
            self.red_frames = self.red_frames+1 if consistent else 1
            self.bank, self.last_red, self.green_frames = bank, stamp_s, 0
            self.observed = 'red4_confirmed' if self.red_frames >= self.config.red_confirmations else 'red4_candidate'
            return False
        if blobs:
            # 불완전 빨강/가림을 소등으로 해석하지 않는다.
            self._forget_pattern()
            self.observed = 'red_partial_or_ambiguous'
            return False
        if self.bank is None or self.red_frames < self.config.red_confirmations:
            self.observed = 'waiting_red4'
            return False
        if stamp_s-self.last_red > self.config.max_red_to_green_s:
            self._forget_pattern()
            self.observed = 'transition_expired'
            return False
        bank = self.bank
        half_x = bank['diameter']*.85
        half_y = bank['diameter']*.85+bank['vertical']
        # 예측 광원이 화면에 전부 들어오지 않으면 부분 영상으로 확정하지 않는다.
        if not (0 <= bank['gx']-half_x and bank['gx']+half_x < rgb.shape[1] and
                0 <= bank['gy']-half_y and bank['gy']+half_y < rgb.shape[0]*.58):
            self.green_frames = 0
            self.observed = 'green_roi_outside'
            return False
        green = cv2.inRange(hsv, (41, 131, 176), (89, 255, 255))
        candidates = [b for b in _blobs(green) if abs(b[0]-bank['gx']) <= half_x and
                      abs(b[1]-bank['gy']) <= half_y and .5*bank['diameter'] <= b[2] <= 1.8*bank['diameter']]
        detected = len(candidates) == bank['pairs']
        if detected and bank['pairs'] == 2:
            detected = (abs(candidates[0][0]-candidates[1][0]) <= .6*bank['diameter'] and
                        abs(abs(candidates[0][1]-candidates[1][1])-2*bank['vertical']) <= .5*bank['diameter'])
        self.green_frames = self.green_frames+1 if detected else 0
        self.started = self.green_frames >= self.config.green_confirmations
        self.observed = 'green_confirmed' if self.started else ('green_candidate' if detected else 'waiting_green')
        return self.started
