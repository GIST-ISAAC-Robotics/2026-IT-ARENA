"""가로 배열의 실제 정지 픽셀을 기존 4열 검출기와 비교한다. 실제 출발 시험 아님."""
from pathlib import Path
import hashlib
import json
import sys

import cv2

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.core import red_signal_mask
from arena_autonomy.race_start import OfficialStartSignal, _blobs, _red_bank


def main():
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/core.py',
                        ROOT/'src/arena_autonomy/arena_autonomy/race_start.py')}
    rows = []
    for name in ('rear_20cm', 'rear_30cm'):
        frames = {}
        pixels = []
        for stage in ('red4', 'green'):
            capture = OUT/f'horizontal_{stage}'/'capture'
            report = json.loads((capture/'report.json').read_text())
            assert report['capture_complete'] and report['source_unchanged'], capture
            path = capture/(name+'.png')
            bgr = cv2.imread(str(path))
            assert bgr is not None, path
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            frames[stage] = rgb
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            red = red_signal_mask(hsv)
            red[int(rgb.shape[0]*.58):] = 0
            green = cv2.inRange(hsv, (41, 131, 176), (89, 255, 255))
            green[int(rgb.shape[0]*.58):] = 0
            blobs = _blobs(red)
            pixels.append(dict(stage=stage, red_blobs=blobs, red_bank=_red_bank(blobs),
                               green_blobs=_blobs(green), capture_clean=report['passed'],
                               file=path.relative_to(ROOT).as_posix()))
            hashes[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
        detector = OfficialStartSignal()
        replay = []
        # 각 단계는 같은 정지 프레임을 반복한 분류 비교다. 실시간 전환/노출 증거가 아니다.
        for stage in ('green',)*3+('red4',)*3+('green',)*3:
            stamp = len(replay)*.1
            detector.update(frames[stage], stamp)
            replay.append(dict(stage=stage, assigned_stamp=stamp,
                               started=detector.started, observed=detector.observed))
        rows.append(dict(mount=name, pixels=pixels, replay=replay,
                         green_alone_started=any(r['started'] for r in replay[:3]),
                         red_then_green_started=detector.started))
    result = dict(scope='static captured pixels repeated with assigned timestamps, not live start',
                  source_sha256=hashes, rows=rows,
                  limitations=['retained provisional 1.2 m beam height and single emitters',
                               'no real exposure, live sequence, motion, or occlusion proof'])
    with (OUT/'pixel_analysis.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps([dict(mount=r['mount'], green_alone=r['green_alone_started'],
                           red_then_green=r['red_then_green_started'],
                           states=[dict(stage=s['stage'], reds=len(s['red_blobs']),
                                        greens=len(s['green_blobs']), bank=s['red_bank'] is not None)
                                   for s in r['pixels']]) for r in rows]))


if __name__ == '__main__':
    main()
