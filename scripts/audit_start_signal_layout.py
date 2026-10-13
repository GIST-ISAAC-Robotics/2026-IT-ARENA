#!/usr/bin/env python3
"""평가용 RGB 합성: 현행 임시 3등 검출기의 공식 5세트 배열 적합성.

픽셀 크기/위치는 시험 변수이며 공식 설치 높이·렌즈/노출·실물 검증이 아니다.
공식 시퀀스는 영상 생성에만 사용한다. 검출기는 RGB 픽셀만 받는다.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from arena_autonomy.core import StartSignal


def light_image(reds=0, green=False, diameter=12, pitch_ratio=2., paired=False,
                offset=(170, 125), green_shift=0, red_scale=None):
    rgb = np.full((480, 848, 3), 100, dtype=np.uint8)
    x0, y0 = offset
    pitch = diameter*pitch_ratio
    pair_step = diameter+max(1, round(diameter/3))
    for i in range(5):
        lit = (i < reds) if i < 4 else green
        color = (255, 4, 1) if i < 4 else (1, 255, 10)
        color = color if lit else (5, 5, 5)
        size = diameter if red_scale is None or i == 4 else diameter*red_scale[i]
        for row in range(2 if paired else 1):
            cx = round(x0+i*pitch+(green_shift if i == 4 else 0))
            cy = round(y0+row*pair_step)
            cv2.circle(rgb, (cx, cy), max(1, round(size/2)), color, -1)
    return rgb


def evaluate(detector_factory=StartSignal):
    results = []
    for diameter in (5, 8, 12, 20, 28):
        for paired in (False, True):
            detector = detector_factory()
            early = []
            # 시간표는 생성기에만 있다. 검출기에 경과 시간으로 만든 go를 전달하지 않는다.
            for red in (0, 1, 2, 3, 4):
                for _ in range(10):
                    early.append(bool(detector.update(light_image(red, diameter=diameter, paired=paired))))
            first_green = None
            for frame in range(50):
                started = detector.update(light_image(0, True, diameter, paired=paired))
                if started and first_green is None:
                    first_green = frame
            latched_after_off = bool(detector.update(light_image(diameter=diameter, paired=paired)))
            results.append(dict(diameter_px=diameter, pitch_ratio=2., paired=paired,
                                false_start=any(early), first_green_frame=first_green,
                                started_after_green=bool(detector.started),
                                latched_after_off=latched_after_off))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rows = evaluate()
    result = dict(scope='synthetic_pixels_not_real_or_gazebo', rows=rows,
                  detected_cases=sum(r['started_after_green'] for r in rows),
                  false_start_cases=sum(r['false_start'] for r in rows),
                  code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
