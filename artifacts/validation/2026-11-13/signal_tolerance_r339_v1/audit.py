"""저장 RGB의 광학 민감도. 실물 조명·오출발률 측정이나 계수 교정이 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.race_start_local import LocalOfficialStartSignal
from arena_autonomy.race_start_marker import MarkerAnchoredStartSignal

SOURCE = A.parent/'old_signal_610_r326_v1/render_current'


def transform(image, name):
    if name == 'baseline':
        return image.copy()
    if name.startswith('gain_'):
        return np.clip(image.astype(float)*float(name[5:]), 0, 255).astype(np.uint8)
    if name.startswith('hue_'):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        hsv[:, :, 0] = (hsv[:, :, 0].astype(int)+int(name[4:])) % 180
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    if name == 'saturation_0.6':
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        hsv[:, :, 1] = np.rint(hsv[:, :, 1]*.6).astype(np.uint8)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    if name == 'blur_1px':
        return cv2.GaussianBlur(image, (5, 5), 1.)
    raise ValueError(name)


def sequence(detector, images):
    frames = [images['off']]*3+[images['red4']]*3+[images['green']]*5
    result = []
    for i, frame in enumerate(frames):
        started = bool(detector.update(frame, 1.+i*.1))
        result.append(dict(stamp_s=1.+i*.1, started=started, reason=detector.observed))
    return dict(started=result[-1]['started'], early_start=any(r['started'] for r in result[:6]),
                first_start_s=next((r['stamp_s'] for r in result if r['started']), None), states=result)


def main():
    assert not (A/'report.json').exists(), 'preserve earlier results'
    variants = ['baseline', 'gain_0.60', 'gain_0.75', 'gain_1.25',
                'hue_-8', 'hue_8', 'saturation_0.6', 'blur_1px']
    paths = sorted(SOURCE.glob('*_red4.png'))
    assert len(paths) == 18
    rows, negative, sources = [], [], {}
    for red in paths:
        name = red.name.removesuffix('_red4.png')
        images = {}
        for state in ('off', 'red4', 'green'):
            path = SOURCE/f'{name}_{state}.png'
            sources[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
            images[state] = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        for variant in variants:
            changed = {state: transform(image, variant) for state, image in images.items()}
            for mode, detector in [('normal', LocalOfficialStartSignal()), ('marker_late', MarkerAnchoredStartSignal())]:
                rows.append(dict(case=name, variant=variant, mode=mode, **sequence(detector, changed)))
        # 렌더에서 점등된 녹색 픽셀 영역에만 유색 소등 렌즈를 가정한다.
        # 밝기 수준은 실물 측정이 아니며 미확인 광학 전제의 반례 탐색이다.
        delta = images['green'].astype(int)-images['off'].astype(int)
        lamp = ((delta[:, :, 1] > 35) & (images['green'][:, :, 1].astype(int) >
                images['green'][:, :, 0].astype(int)+50))
        for value in (100, 150, 200):
            unlit = images['off'].copy()
            unlit[lamp] = (0, value, 0)
            for mode, detector in [('normal', LocalOfficialStartSignal()), ('marker_late', MarkerAnchoredStartSignal())]:
                outcome = sequence(detector, dict(off=unlit, red4=unlit, green=unlit))
                negative.append(dict(case=name, mode=mode, assumed_unlit_green_v=value,
                                     known_render_lamp_pixels=int(lamp.sum()), **outcome))
    summary = {}
    for mode in ('normal', 'marker_late'):
        summary[mode] = {variant: dict(success=sum(r['started'] for r in rows if r['mode']==mode and r['variant']==variant),
            total=18, early=sum(r['early_start'] for r in rows if r['mode']==mode and r['variant']==variant)) for variant in variants}
    report = dict(scope='static saved render perturbation; repeated pixels assigned synthetic distinct times; no real exposure/PWM/noise validation',
        variants=variants, summary=summary,
        negative_false_start_counts=dict(Counter(f"{r['mode']}_v{r['assumed_unlit_green_v']}" for r in negative if r['started'])),
        negative_total=len(negative), sources=sources,
        unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==s for p, s in sources.items()),
        detector_changed=False, rows=rows, negative_rows=negative)
    (A/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: report[k] for k in ('summary', 'negative_false_start_counts', 'negative_total', 'unchanged')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
