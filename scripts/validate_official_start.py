#!/usr/bin/env python3
"""B3 후보의 합성 영상 거리/열화·오출발 시험. 실물 광학/시야 검증 아님."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from arena_autonomy.race_start import OfficialStartSignal, StartConfig


def image(reds, green, *, distance=1., width=1280, height=720, fov=75., paired=True,
          blur=0., gain=1., occlude=False, green_shift=0., simultaneous=False):
    focal = width/(2*math.tan(math.radians(fov)/2))
    diameter = focal*.015/distance  # 시험 가정: 한 광원 15 mm, 실물 해석 미확정
    pitch = focal*.030/distance
    pair_pitch = focal*.020/distance
    scale = 4
    canvas = np.full((height*scale, width*scale, 3), 70, np.uint8)
    for i in range(5):
        lit = i < reds if i < 4 else green
        if simultaneous and green and i < 4:
            lit = True
        color = ((255, 4, 1) if i < 4 else (1, 255, 10)) if lit else (4, 4, 4)
        for j in range(2 if paired else 1):
            x = width*.35+i*pitch+(green_shift*diameter if i == 4 else 0)
            y = height*.28+j*pair_pitch
            if occlude and i == 2:
                color = (70, 70, 70)
            cv2.circle(canvas, (round(x*scale), round(y*scale)), max(1, round(diameter*scale/2)), color, -1)
    result = cv2.resize(canvas, (width, height), interpolation=cv2.INTER_AREA)
    if blur:
        result = cv2.GaussianBlur(result, (0, 0), blur)
    return np.clip(result.astype(float)*gain, 0, 255).astype(np.uint8)


def run_case(name, *, expected, config, camera, stages=None, **kwargs):
    signal = OfficialStartSignal(config)
    # 생성기에만 명목 시간표를 둔다. detector에는 RGB·촬영 시각만 입력한다.
    stages = stages or [(0, False, 10), (1, False, 10), (2, False, 10),
                        (3, False, 10), (4, False, 10), (0, True, 50), (0, False, 3)]
    t, start_time, expected_go, costs, early = 0., None, None, [], False
    active_costs = []
    stage_results = []
    for red, green, frames in stages:
        if green and expected_go is None:
            expected_go = t
        rgb = image(red, green, **camera, **kwargs)
        for _ in range(frames):
            previously_started = signal.started
            before = time.perf_counter()
            started = signal.update(rgb, round(t, 7))
            costs.append((time.perf_counter()-before)*1000)
            if not previously_started:
                active_costs.append(costs[-1])
            if started and start_time is None:
                start_time = t
            early |= started and expected_go is None
            t += .1
        stage_results.append(dict(red_count=red, green=green, observed=signal.observed,
                                  started=signal.started))
    detected = start_time is not None
    passed = (not early and (detected if expected == 'start' else not detected)) if expected != 'exploratory' else None
    return dict(name=name, expected=expected, passed=passed, started=detected, false_start=early,
                first_start_s=start_time, expected_green_s=expected_go,
                detection_delay_s=None if start_time is None or expected_go is None else start_time-expected_go,
                mean_processing_ms=float(np.mean(costs)), max_processing_ms=float(np.max(costs)),
                prestart_mean_processing_ms=float(np.mean(active_costs)),
                prestart_p95_processing_ms=float(np.percentile(active_costs, 95)),
                inputs=dict(camera, **kwargs), stages=stage_results)


def validate():
    root = Path(__file__).resolve().parents[1]
    cfg_path = root/'src/arena_description/config/b0183_c1.yaml'
    cfg = yaml.safe_load(cfg_path.read_text(encoding='utf-8'))['camera']
    camera = dict(width=cfg['width_px'], height=cfg['height_px'], fov=cfg['horizontal_fov_deg'])
    config = StartConfig()
    rows = []
    for distance in (.5, 1., 1.5, 2., 2.5, 3., 4., 6.):
        for paired in (False, True):
            rows.append(run_case(f'distance_{distance}_paired_{paired}', expected='start' if distance <= 1. else 'exploratory',
                                 config=config, camera=camera, distance=distance, paired=paired))
    for blur, gain in ((1., 1.), (2., 1.), (0., .6), (0., .8)):
        rows.append(run_case(f'degradation_{blur}_{gain}', expected='exploratory', config=config,
                             camera=camera, blur=blur, gain=gain))
    for name, kwargs, stages in [
        ('green_only', {}, [(0, True, 30)]),
        ('all_dark', {}, [(0, False, 40)]),
        ('partial_occlusion', {'occlude': True}, None),
        ('simultaneous_red_green', {'simultaneous': True}, None),
        ('unrelated_green', {'green_shift': 3.}, None),
    ]:
        rows.append(run_case(name, expected='no_start', config=config, camera=camera, stages=stages, **kwargs))
    for width, height in ((1920, 1080),):
        rows.append(run_case('resolution_diagnostic_3m', expected='exploratory', config=config,
                             camera=dict(camera, width=width, height=height), distance=3.))
    return dict(scope='supersampled_pinhole_pixels_not_physical_visibility_or_camera_test',
                camera_source_sha256=hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
                candidate_config=asdict(config), source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                assumptions=['15 mm per individual emitter is provisional', '30 mm horizontal pitch',
                             '20 mm paired emitter pitch', 'centered visible housing, no lens distortion or noise',
                             '10 Hz visual processing, not camera acquisition limit',
                             '1920x1080 row is a diagnostic, no runtime resolution change'],
                passed=all(r['passed'] is not False for r in rows), cases=len(rows), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = validate()
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
