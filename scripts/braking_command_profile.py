"""제동 대조의 사전 고정 요청. 위치/속도 정답 피드백으로 명령을 바꾸지 않는다."""
from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class CommandProfile:
    samples: tuple
    duration_s: float
    brake_window_s: tuple
    source: dict

    def at(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('nonfinite command time')
        if elapsed < 0 or elapsed >= self.duration_s:
            return 0., 0.
        index = bisect_right([row[0] for row in self.samples], elapsed)-1
        return self.samples[index][1:]


def load_profile(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema_version') != 1 or data.get('interpolation') != 'previous_sample':
        raise ValueError('unsupported command profile')
    rows = tuple(tuple(row) for row in data['samples_t_speed_steering'])
    if (len(rows) < 3 or any(len(row) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) for v in row) for row in rows)):
        raise ValueError('invalid profile rows')
    if rows[0] != (0., 0., 0.) or rows[-1][1:] != (0., 0.):
        raise ValueError('profile must start and end at zero command')
    if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError('profile time must increase')
    duration = data['duration_s']
    if not math.isfinite(duration) or not 0 < duration <= 10 or duration != rows[-1][0]:
        raise ValueError('invalid profile duration')
    if any(not 0 <= speed <= 1.000001 or abs(steering) > .37 for _, speed, steering in rows):
        raise ValueError('profile exceeds registered low speed or steering bound')
    window = tuple(data['brake_window_s'])
    if len(window) != 2 or not all(math.isfinite(v) for v in window) or not 0 <= window[0] < window[1] <= duration:
        raise ValueError('invalid brake window')
    return CommandProfile(rows, float(duration), window, data.get('source', {}))


def audit_requests(rows, profile, start, end):
    active = [r for r in rows if start <= r['sim_s'] <= end]
    ordered = bool(active) and all(b['sim_s'] > a['sim_s'] for a, b in zip(active, active[1:]))
    gap = max((b['sim_s']-a['sim_s'] for a, b in zip(active, active[1:])), default=None)
    matching = bool(active) and all(
        abs(r['speed_mps']-profile.at(r['sim_s']-start)[0]) < 1e-8 and
        abs(r['steering_rad']-profile.at(r['sim_s']-start)[1]) < 1e-8 for r in active)
    checks = {'ordered': ordered, 'matches_registered_profile': matching,
              'covers_start': bool(active) and active[0]['sim_s']-start <= .05,
              'covers_end': bool(active) and end-active[-1]['sim_s'] <= .05,
              'bounded_publish_gap': gap is not None and gap <= .05}
    return {'checks': checks, 'passed': all(checks.values()), 'samples': len(active), 'max_gap_s': gap}
