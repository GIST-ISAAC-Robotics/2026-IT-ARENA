"""보정 전 차량 마커 관측. 거리/동일 차로/빈 공간 또는 주행 허가를 반환하지 않는다."""
from collections import deque
import math

import numpy as np


COURSE_AND_SELF_IDS = frozenset((0, 20, 30, 45, 49))


def participant_ids(values):
    """참가 차량으로 확인된 ID만 명시한다. 남는 ID를 차량으로 추정하지 않는다."""
    ids = tuple(sorted(set(values)))
    if not ids or any(type(value) is not int or not 0 <= value < 50 for value in ids):
        raise ValueError('vehicle marker IDs must be explicit DICT_4X4_50 integers')
    if COURSE_AND_SELF_IDS.intersection(ids):
        raise ValueError('course and own marker IDs cannot be vehicle targets')
    return ids


def summarize_markers(observations, width, height, allowed_ids):
    """기존 검출 결과를 작은 정규화 관측으로 축약한다. 추가 검출/영상 복사는 없다."""
    allowed = set(allowed_ids)
    if not allowed:
        return []
    if min(width, height) <= 0:
        raise ValueError('invalid image size')
    found = {}
    for identity, corners in observations:
        if identity not in allowed:
            continue
        p = np.asarray(corners, dtype=float)
        if (p.shape != (4, 2) or not np.isfinite(p).all() or (p < 0).any()
                or (p[:, 0] >= width).any() or (p[:, 1] >= height).any()):
            continue
        edges = np.roll(p, -1, axis=0)-p
        lengths = np.linalg.norm(edges, axis=1)
        cross = edges[:, 0]*np.roll(edges[:, 1], -1)-edges[:, 1]*np.roll(edges[:, 0], -1)
        if lengths.min() < 6 or lengths.max()/lengths.min() > 3 or not (np.all(cross > 0) or np.all(cross < 0)):
            continue
        center, size = p.mean(axis=0), float(lengths.mean())
        found.setdefault(identity, []).append(dict(id=int(identity),
            center_x=float(center[0]/width), center_y=float(center[1]/height),
            edge_fraction=size/width))
    output = []
    for identity, matches in sorted(found.items()):
        ref = matches[0]
        # 0.02 검출기에서는 한 코드의 중복 사각형이 생길 수 있다. 서로 다른 크기나
        # 위치의 같은 ID를 섞어 접근률로 해석하지 않고 해당 프레임을 무효로 남긴다.
        equivalent = all(
            abs(math.log(item['edge_fraction']/ref['edge_fraction'])) <= math.log(1.15)
            and math.hypot(item['center_x']-ref['center_x'],
                           (item['center_y']-ref['center_y'])*height/width) <= .12*ref['edge_fraction']
            for item in matches)
        output.append(dict(ref, duplicates=len(matches), ambiguous=not equivalent))
    return output


class VehicleMarkerObserver:
    """채택된 새 영상만 누적한다. 소실은 미관측이며 안전/차량 없음 판정이 아니다."""
    def __init__(self, ids, max_age_s=.35, window_s=.8):
        self.ids = participant_ids(ids)
        if not 0 < max_age_s < window_s:
            raise ValueError('invalid vehicle observation window')
        self.max_age_s, self.window_s = max_age_s, window_s
        self.reset()

    def reset(self):
        self.stamp = None
        self.tracks = {}
        self.observations = []
        self.image_shape = None

    def update(self, stamp, observations, image_shape):
        if not math.isfinite(stamp) or (self.stamp is not None and stamp <= self.stamp):
            return False
        shape = tuple(image_shape)
        if len(shape) != 2 or min(shape) <= 0:
            raise ValueError('invalid marker image shape')
        if self.image_shape != shape or (self.stamp is not None and stamp-self.stamp > self.max_age_s):
            self.tracks.clear()
        self.image_shape, self.stamp = shape, stamp
        current = {int(row['id']): row for row in observations if row['id'] in self.ids}
        self.tracks = {key: value for key, value in self.tracks.items()
                       if key in current and not current[key]['ambiguous']}
        result = []
        for identity, row in sorted(current.items()):
            item = dict(row, samples=0, observed_span_s=0., expansion_per_s=None,
                        inverse_expansion_s=None, growing=False, bearing_stable=False)
            if not row['ambiguous']:
                track = self.tracks.setdefault(identity, deque(maxlen=10))
                track.append((stamp, row['edge_fraction'], row['center_x'], row['center_y']))
                while stamp-track[0][0] > self.window_s:
                    track.popleft()
                span = stamp-track[0][0]
                item.update(samples=len(track), observed_span_s=span)
                if len(track) >= 3 and span >= .2-1e-9:
                    values = np.asarray(track)
                    times = values[:, 0]-values[0, 0]
                    logs = np.log(values[:, 1])
                    centered = times-times.mean()
                    slope = float(np.dot(centered, logs-logs.mean())/np.dot(centered, centered))
                    residual = float(np.max(np.abs(logs-(logs.mean()+slope*centered))))
                    stable = float(np.ptp(values[:, 2])) <= .025 and float(np.ptp(values[:, 3])) <= .04
                    growing = slope > .15 and logs[-1]-logs[0] >= math.log(1.10) and residual < .08
                    item.update(expansion_per_s=slope, inverse_expansion_s=1/slope if slope > .01 else None,
                                bearing_stable=stable, growing=growing)
            result.append(item)
        self.observations = result
        return True

    def snapshot(self, now):
        age = None if self.stamp is None else now-self.stamp
        fresh = age is not None and math.isfinite(age) and 0 <= age < self.max_age_s
        return dict(mode='observation_only', source_stamp_s=self.stamp,
                    fresh=fresh, observations=self.observations if fresh else [],
                    no_detection_is_clear=False, metric_range_available=False,
                    controls_speed=False)
