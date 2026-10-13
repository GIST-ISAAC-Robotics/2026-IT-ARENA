#!/usr/bin/env python3
"""B3 시각 시간축/단일 작업 큐 대조. 가상 지연은 PC/Jetson 성능 측정이 아니다."""
import argparse
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import zipfile

import cv2
import numpy as np

from arena_autonomy.vision_process import VisionPipeline, recognize
from audit_start_signal_layout import light_image

ROOT = Path(__file__).resolve().parents[1]
CAPTURES = ROOT/'artifacts/validation/2026-10-13/race_start_render_v2/run'


@dataclass(frozen=True)
class Scenario:
    name: str
    expected: str = 'start'
    red3_s: float = 1.
    red4_s: float = 1.
    work_delay_s: float = .02
    camera_hz: float = 60.
    fault: str = ''
    grid: int | None = None

    @property
    def green_s(self):
        return 3.+self.red3_s+self.red4_s


def stage(t, case):
    if t < 1.:
        return 0, False
    if t < 2.:
        return 1, False
    if t < 3.:
        return 2, False
    if t < 3.+case.red3_s:
        return 3, False
    if t < case.green_s:
        return 4, False
    return 0, t < case.green_s+5.


def load_images(grid=None):
    frames = {(n, False): light_image(n, diameter=12, paired=True) for n in range(5)}
    frames[0, True] = light_image(0, True, diameter=12, paired=True)
    if grid is not None:
        for key, name in [((0, False), 'off'), ((4, False), 'red4'), ((0, True), 'green')]:
            path = CAPTURES/f'grid{grid}_{name}.png'
            bgr = cv2.imread(str(path))
            if bgr is None:
                raise ValueError(f'missing captured input {path}')
            frames[key] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        # 촬영하지 않은 부분 점등을 실측 영상으로 만들지 않는다. red1..3은 off로 대체.
        for n in (1, 2, 3):
            frames[n, False] = frames[0, False]
    return frames


def message(stamp, rgb):
    sec, nanosec = divmod(round(stamp*1e9), 1_000_000_000)
    h, w = rgb.shape[:2]
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
                           height=h, width=w, step=w*3, encoding='rgb8', data=rgb.tobytes())


class TimedWorker:
    """인식은 실제 코드, 결과 배송만 가상 시각으로 지연시킨다. 실측 처리율은 아니다."""
    def __init__(self, delay):
        self.delay, self.now = delay, 0.
        self.due, self.result = None, None

    def submit(self, request, data):
        if self.result is not None:
            raise RuntimeError('more than one inflight request')
        self.result = recognize(request, data)
        self.due = self.now+self.delay

    def poll(self):
        if self.result is None or self.now+1e-9 < self.due:
            return None
        result, self.result = self.result, None
        return result


def run_case(case, images=None):
    if case.expected not in ('start', 'no_start', 'exploratory'):
        raise ValueError('invalid expectation')
    if not 1 <= case.camera_hz <= 120 or not 0 < case.work_delay_s < 2:
        raise ValueError('invalid timing fixture')
    images = load_images(case.grid) if images is None else images
    worker = TimedWorker(case.work_delay_s)
    pipe = VisionPipeline(worker, signal_mode='official_4red_candidate')
    next_frame, reset_done = 0, False
    start_time, start_capture, early = None, None, False
    transitions, accepted_stamps = [], []
    previous = None
    # 5 ms is evaluation scheduling resolution, not MCU/Gazebo step or processing target.
    for tick in range(math.ceil((case.green_s+5.5)/.005)+1):
        t = round(tick*.005, 9)
        worker.now = t
        if case.fault == 'reset_on_green' and t >= case.green_s and not reset_done:
            pipe.reset()
            reset_done = True
        while next_frame/case.camera_hz <= t+1e-9:
            stamp = next_frame/case.camera_hz
            next_frame += 1
            key = stage(stamp, case)
            if case.fault == 'drop_transition' and case.green_s-.15 <= stamp <= case.green_s+.35:
                continue
            if case.fault == 'drop_red4' and key == (4, False):
                continue
            if case.fault == 'green_only':
                key = (0, True)
            if case.fault == 'no_green' and key == (0, True):
                key = (0, False)
            if case.fault == 'green_flash' and stamp >= case.green_s+.1:
                key = (0, False)
            if case.fault == 'frozen_green_stamp' and key == (0, True):
                stamp = case.green_s
            frame = message(stamp, images[key])
            pipe.offer(frame, wall_now=t)
            if case.fault == 'duplicates':
                pipe.offer(frame, wall_now=t)
                pipe.offer(frame, wall_now=t)
            if case.fault == 'old_frame' and stamp > .5:
                old_stamp = stamp-.5
                pipe.offer(message(old_stamp, images[stage(old_stamp, case)]), wall_now=t)
        accepted = pipe.pump(t, wall_now=t)
        if accepted:
            accepted_stamps.append(accepted['stamp'])
        status = (pipe.signal.observed, pipe.signal.started, pipe.fault)
        if status != previous:
            transitions.append(dict(time_s=t, capture_s=None if accepted is None else accepted['stamp'],
                                    observed=status[0], started=status[1], fault=status[2]))
            previous = status
        early |= pipe.signal.started and t < case.green_s
        if pipe.signal.started and start_time is None:
            start_time, start_capture = t, pipe.last_accepted
    started = start_time is not None
    intervals = np.diff(accepted_stamps)
    passed = None if case.expected == 'exploratory' else bool(not early and started == (case.expected == 'start'))
    return dict(scenario=asdict(case), passed=passed, started=started, false_start=early,
                green_s=case.green_s, first_accepted_start_s=start_time, first_start_capture_s=start_capture,
                decision_delay_s=None if start_time is None else start_time-case.green_s,
                off_keeps_start=bool(started and pipe.signal.started), fault=pipe.fault,
                accepted_gap_max_s=float(max(intervals)) if intervals.size else None,
                stats=pipe.stats, transitions=transitions)


def scenarios():
    cases = [Scenario(f'nominal_r3_{a}_r4_{b}', red3_s=a, red4_s=b) for a in (1., 2., 3.) for b in (1., 2., 3.)]
    cases += [Scenario(f'latency_{delay}', expected='start' if delay <= .15 else 'exploratory', work_delay_s=delay)
              for delay in (.08, .15, .25, .30, .40)]
    cases += [Scenario(fault, expected='start' if fault in ('duplicates', 'old_frame') else 'no_start', fault=fault)
              for fault in ('duplicates', 'old_frame', 'drop_transition', 'drop_red4', 'green_only',
                            'no_green', 'green_flash', 'frozen_green_stamp', 'reset_on_green')]
    cases += [Scenario('worker_watchdog', expected='no_start', work_delay_s=1.1)]
    cases += [Scenario(f'captured_grid_{grid}_{delay}', expected='start' if delay <= .15 else 'exploratory',
                       grid=grid, work_delay_s=delay) for grid in (2, 3) for delay in (.02, .15, .3)]
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    if not target.is_relative_to(ROOT/'artifacts') or target == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    target.mkdir(parents=True, exist_ok=False)
    paths = [Path(__file__).resolve(), ROOT/'scripts/audit_start_signal_layout.py',
             *(ROOT/'src/arena_autonomy/arena_autonomy'/name for name in ('vision_process.py', 'race_start.py', 'core.py')),
             ROOT/'tests/test_start_signal_timing.py',
             *(CAPTURES/f'grid{grid}_{stage}.png' for grid in (2, 3) for stage in ('off', 'red4', 'green'))]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(target/'source_input_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            bundle.write(path, path.relative_to(ROOT))
    rows = [run_case(case) for case in scenarios()]
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    result = dict(scope='virtual_delivery_clock_actual_pixel_recognition_not_benchmark_or_live_render',
                  source_hashes=hashes, source_unchanged=unchanged,
                  passed=bool(unchanged and all(r['passed'] is not False and not r['false_start'] for r in rows)),
                  cases=len(rows), required=sum(r['passed'] is not None for r in rows),
                  assumptions=['60 Hz nominal input, actual pipeline throttles to about 10 Hz',
                               'worker delivery delays are invented scenarios, not Jetson measurements',
                               '5 ms evaluation scheduler, no ROS/Gazebo/MCU execution',
                               'captured inputs are static diagnostic scenes; red1..3 replaced by off',
                               'official random-stage durations sampled at 1, 2, 3 seconds, not every real value'], rows=rows)
    (target/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('passed', 'cases', 'required', 'source_unchanged')}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
