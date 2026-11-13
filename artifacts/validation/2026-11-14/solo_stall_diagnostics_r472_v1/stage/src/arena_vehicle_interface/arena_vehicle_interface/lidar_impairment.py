"""시험 전용 거리오차/전송지연. 취득 시각을 바꾸거나 결측을 자유 공간으로 만들지 않는다."""
from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ImpairmentConfig:
    mode: str = 'none'
    amplitude_m: float = 0.
    bias_m: float = 0.
    delay_s: float = 0.
    jitter_s: float = 0.
    seed: int = 22

    def __post_init__(self):
        if self.mode not in ('none', 'uniform', 'gaussian'):
            raise ValueError('unknown lidar error mode')
        for field in ('amplitude_m', 'bias_m', 'delay_s', 'jitter_s'):
            if not math.isfinite(getattr(self, field)):
                raise ValueError('nonfinite impairment')
        if not 0 <= self.amplitude_m <= .2 or abs(self.bias_m) > .2:
            raise ValueError('test range error outside limit')
        if not 0 <= self.jitter_s <= self.delay_s <= .5:
            raise ValueError('require 0 <= jitter <= delay <= 0.5 s')
        if self.mode == 'none' and self.amplitude_m != 0:
            raise ValueError('none mode cannot have random error')
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or not 0 <= self.seed < 2**32:
            raise ValueError('seed must be uint32')


class ScanImpairment:
    def __init__(self, config):
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        self.queue = deque()
        self.count = self.invalidated = self.scans = 0
        self.total = self.square = self.maximum = 0.
        self.last_source = None

    def distort(self, ranges, minimum, maximum):
        original = np.asarray(ranges, dtype=float)
        result = original.copy()
        valid = np.isfinite(original) & (original >= minimum) & (original <= maximum)
        count = int(np.sum(valid))
        cfg = self.config
        if cfg.mode == 'uniform':
            errors = self.rng.uniform(-cfg.amplitude_m, cfg.amplitude_m, count)
        elif cfg.mode == 'gaussian':
            errors = self.rng.normal(0., cfg.amplitude_m, count)
        else:
            errors = np.zeros(count)
        errors += cfg.bias_m
        result[valid] += errors
        invalid = valid & ((result < minimum) | (result > maximum))
        result[invalid] = np.nan
        self.count += count
        self.invalidated += int(np.sum(invalid))
        self.total += float(np.sum(errors))
        self.square += float(errors @ errors)
        self.maximum = max(self.maximum, float(np.max(np.abs(errors))) if count else 0.)
        self.scans += 1
        return result.tolist()

    def advance(self, source_stamp):
        if self.last_source is not None and source_stamp < self.last_source:
            self.queue.clear()  # reset 전에 취득한 스캔을 새 시간축에 발행하지 않는다.
        self.last_source = source_stamp
        ready = []
        while self.queue and self.queue[0][0] <= source_stamp + 1e-9:
            due, acquired, scan, audit = self.queue.popleft()
            ready.append((scan, dict(audit, completion_stamp_s=acquired,
                requested_release_stamp_s=due, release_source_stamp_s=source_stamp,
                injected_delay_s=source_stamp-acquired, impairment=self.summary())))
        return ready

    def enqueue(self, scan, source_stamp, audit):
        cfg = self.config
        delay = cfg.delay_s + self.rng.uniform(-cfg.jitter_s, cfg.jitter_s)
        due = max(source_stamp + delay, self.queue[-1][0] if self.queue else source_stamp)
        if len(self.queue) >= 100:
            raise RuntimeError('bounded lidar delay queue overflow')
        self.queue.append((due, source_stamp, scan, audit))

    def summary(self):
        mean = self.total / self.count if self.count else 0.
        return {'config': asdict(self.config), 'samples': self.count, 'scans': self.scans,
                'mean_error_m': mean,
                'rms_error_m': math.sqrt(self.square / self.count) if self.count else 0.,
                'max_abs_error_m': self.maximum, 'invalidated': self.invalidated,
                'scope': 'synthetic radial error; not manufacturer distribution or measured transport'}
