"""보존한5대 정지 프레임에서 기존/지역 신호 후보를 동일 입력으로 비교한다."""
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

import cv2

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.race_start import OfficialStartSignal
from arena_autonomy.race_start_local import LocalOfficialStartSignal


def main():
    output = BASE/'dense_pixel_candidates_v2.json'
    if output.exists():
        raise FileExistsError(output)
    sources = [Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',
               ROOT/'src/arena_autonomy/arena_autonomy/race_start_local.py']
    images = []
    for stage in ('red4', 'green'):
        p = BASE/f'dense_pixels_{stage}/capture/slot3.png'
        sources.append(p)
        images.append(cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB))
    rows = []
    for detector in (OfficialStartSignal, LocalOfficialStartSignal):
        signal = detector()
        early = [signal.update(images[1], i*.1) for i in range(4)]
        signal.reset()
        states, costs = [], []
        for i, rgb in enumerate([images[0]]*3+[images[1]]*3):
            began = time.perf_counter_ns()
            signal.update(rgb, i*.1)
            costs.append((time.perf_counter_ns()-began)/1e6)
            states.append(signal.observed)
        rows.append(dict(detector=detector.__name__, states=states, started=signal.started,
                         early_start=any(early), median_ms=statistics.median(costs), maximum_ms=max(costs)))
    report = dict(scope='static recorded frames repeated with synthetic times; not live sequence or hardware proof',
                  rows=rows, expected_comparison=(not rows[0]['started'] and rows[1]['started']
                       and not any(r['early_start'] for r in rows)),
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['expected_comparison'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
