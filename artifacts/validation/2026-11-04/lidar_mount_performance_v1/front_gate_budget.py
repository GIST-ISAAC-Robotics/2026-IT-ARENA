"""장착 변동별 전방 ±100° unknown 광선 예산(기하 계산, 주행 아님).

현행 판정선: 전방 결측 합계 6° 이하·전방 표본 5% 이하(bounded_local_v4 연속 3° 초과는 측방 거리 검사로 넘김),
그리고 전방 결측이 하나라도 있으면 부분 관측 경로이며 측정 속도 1.2 m/s 초과 시 불허.
C1 500광선(0.72°), 시작 위상 4가지. 마커는 정후방이라 전방에 없다.
"""
import json, math
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
import importlib.util
spec = importlib.util.spec_from_file_location('sr', HERE/'shadow_replay.py'); sr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sr)
STEP = 2*math.pi/500


def budget(sx, posts, diameter, margin):
    out = []
    for phase in (0, .25, .5, .75):
        angles = -math.pi + (np.arange(500)+phase)*STEP
        front = np.abs(angles) <= math.radians(100)
        hit = sr.in_sectors(angles, sr.sectors(sx, posts, diameter, margin, marker=False)) & front
        ids = np.flatnonzero(hit)
        runs = np.split(ids, np.flatnonzero(np.diff(ids) > 1)+1) if len(ids) else []
        out.append(dict(total_deg=round(len(ids)*math.degrees(STEP), 2),
                        widest_deg=round(max((len(r) for r in runs), default=0)*math.degrees(STEP), 2)))
    worst = max(out, key=lambda r: r['total_deg'])
    return dict(**worst, any_front_unknown=worst['total_deg'] > 0,
                passes_6deg_budget=worst['total_deg'] <= 6+1e-7 and worst['total_deg']/math.degrees(STEP) <= .05*278)


rows = []
for layout in ('jr', 'c'):
    L = sr.LAYOUTS[layout]
    for label, sx, posts in (('nominal', L['sx'], L['posts']),
                             ('sensor_x_-10mm', L['sx']-.01, L['posts']), ('sensor_x_+10mm', L['sx']+.01, L['posts']),
                             ('front_posts_x_-10mm', L['sx'], [(x-.01 if i < 2 else x, y) for i, (x, y) in enumerate(L['posts'])]),
                             ('front_posts_x_+10mm', L['sx'], [(x+.01 if i < 2 else x, y) for i, (x, y) in enumerate(L['posts'])])):
        for d in (.0036, .004, .007, .010):
            for margin in (0., .5, 1.):
                rows.append(dict(layout=layout, variant=label, diameter_mm=d*1000, margin_deg=margin, **budget(sx, posts, d, margin)))
(HERE/'front_gate_budget.json').write_text(json.dumps(dict(scope=__doc__, rows=rows), ensure_ascii=False, indent=1), encoding='utf-8')
for r in rows:
    if r['margin_deg'] == .5 or r['diameter_mm'] in (3.6, 4.0):
        print(r)
