"""정지 렌더링 영상의 실제 픽셀/기존 출발 검출기 확인. 폐루프 출발 증거 아님."""
from pathlib import Path
import hashlib
import json
import sys

import cv2

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.core import red_signal_mask, marker_ids
from arena_autonomy.race_start import OfficialStartSignal, _blobs, _red_bank


def main():
    rows = []
    hashes = {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in
              (Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/core.py',ROOT/'src/arena_autonomy/arena_autonomy/race_start.py')}
    for height in (25,40):
        for name in ('rear_20cm','rear_30cm'):
            frames = {}
            states = []
            for stage in ('red4','green'):
                capture = OUT/f'render_h{height}_{stage}'/'capture'
                report = json.loads((capture/'report.json').read_text())
                assert report['capture_complete'] and report['source_unchanged'], capture
                path = capture/(name+'.png')
                bgr = cv2.imread(str(path))
                assert bgr is not None
                rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                frames[stage] = rgb
                hsv = cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
                red = red_signal_mask(hsv)
                red[int(rgb.shape[0]*.58):] = 0
                blobs = _blobs(red)
                green = cv2.inRange(hsv,(41,131,176),(89,255,255))
                green[int(rgb.shape[0]*.58):] = 0
                states.append(dict(stage=stage,red_blobs=blobs,red_bank=_red_bank(blobs),
                    green_blobs=_blobs(green),marker_ids=marker_ids(rgb,min_distance_rate=.02),
                    capture_clean=report['passed'],file=path.relative_to(ROOT).as_posix()))
                hashes[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
            detector = OfficialStartSignal()
            replay = []
            # 정지 화면을 반복한 분류 대조다. 서로 독립적인 시간 표본으로 주장하지 않는다.
            for stage in ('green',)*3+('red4',)*3+('green',)*3:
                stamp = len(replay)*.1
                detector.update(frames[stage],stamp)
                replay.append(dict(stage=stage,assigned_stamp=stamp,started=detector.started,observed=detector.observed))
            rows.append(dict(actor_height_m=height/100,mount=name,pixels=states,replay=replay,
                controller_started=detector.started))
    result = dict(scope='captured_static_pixels_repeated_with_assigned_timestamps_not_live_start',
        source_sha256=hashes,rows=rows,
        limitations=['static frames do not demonstrate temporal exposure or actual start switching',
            'same nominal pinhole camera and assumed box geometry, no real hardware claim'])
    with (OUT/'render_analysis.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps([dict(height=r['actor_height_m'],mount=r['mount'],started=r['controller_started'],
        states=[dict(stage=s['stage'],reds=len(s['red_blobs']),greens=len(s['green_blobs']),bank=s['red_bank'] is not None) for s in r['pixels']]) for r in rows]))


if __name__ == '__main__':
    main()
