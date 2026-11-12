"""평가 전용 유한 폭 광선 단면 비교. 정답을 제어기에 넣지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'tests')]
import validate_g1_traffic as harness


def visual_boxes(root, height=.1346):
    model = root.find('model')
    model_z = float(model.findtext('pose').split()[2])
    result = []
    for item in model.findall('link/visual'):
        x, y, z, roll, pitch, yaw = map(float, item.findtext('pose', '0 0 0 0 0 0').split())
        assert roll == pitch == yaw == 0
        sx, sy, sz = map(float, item.findtext('geometry/box/size').split())
        if abs(height-model_z-z) < sz/2:
            result.append((x-sx/2, y-sy/2, x+sx/2, y+sy/2))
    return result


def ranges(boxes, angles, distance, yaw):
    # 차량 좌표로 광선 원점/방향을 변환한다. 센서-차량 중심 거리이며 차간 거리 아님.
    c, s = math.cos(yaw), math.sin(yaw)
    origin = np.array([-distance*c, distance*s])
    direction = np.column_stack((np.cos(angles-yaw), np.sin(angles-yaw)))
    answer = np.full(len(angles), np.inf)
    for xmin, ymin, xmax, ymax in boxes:
        lo, hi = np.zeros(len(angles)), np.full(len(angles), np.inf)
        for axis, lower, upper in ((0, xmin, xmax), (1, ymin, ymax)):
            d = direction[:, axis]
            parallel = abs(d) < 1e-12
            safe = np.where(parallel, 1., d)
            a, b = (lower-origin[axis])/safe, (upper-origin[axis])/safe
            lo = np.maximum(lo, np.where(parallel, -np.inf, np.minimum(a, b)))
            hi = np.minimum(hi, np.where(parallel, np.inf, np.maximum(a, b)))
            if not lower <= origin[axis] <= upper:
                hi = np.where(parallel, -np.inf, hi)
        answer = np.minimum(answer, np.where((hi >= lo) & (hi > 0), lo, np.inf))
    return answer


def main():
    assert not (A/'report.json').exists(), 'preserve earlier report'
    source = A.parent/'published_route_drive_r338_v1/run1/follow_decisions.jsonl'
    patterns = {}
    with source.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            data = event['data']['input']['scan']['measurements']
            count = len(data['angles_rad'])
            patterns.setdefault(count, dict(angles_rad=data['angles_rad'], acquired_at_s=data['acquired_at_s']))
            if set(patterns) >= {504, 508, 512}:
                break
    assert set(patterns) == {504, 508, 512}
    rows = []
    for outline in ('box', 'recessed_sides', 'thin_upper_blade', 'sparse_upper_posts'):
        cfg = harness.load_config(ROOT/'config/tests/official_traffic_v1.json')
        cfg['actor'].update(visual_outline=outline, height_m=.22, bottom_m=.02)
        sdf = harness.actor_sdf(cfg, (0, 0, 0))
        (A/f'{outline}.sdf').write_text(sdf)
        boxes = visual_boxes(ET.fromstring(sdf))
        for count, pattern in patterns.items():
            angles = np.asarray(pattern['angles_rad'])
            for distance in (.4, .8, 1.2):
                for yaw in (0., math.pi/4, math.pi/2):
                    for phase in range(8):
                        r = ranges(boxes, angles+phase/8*2*math.pi/count, distance, yaw)
                        rows.append(dict(outline=outline, count=count, distance_m=distance,
                            yaw_deg=round(math.degrees(yaw)), phase=phase, hits=int(np.isfinite(r).sum())))
    summary = []
    for outline in ('box', 'recessed_sides', 'thin_upper_blade', 'sparse_upper_posts'):
        for distance in (.4, .8, 1.2):
            selected = [r for r in rows if r['outline']==outline and r['distance_m']==distance]
            hits = [r['hits'] for r in selected]
            summary.append(dict(outline=outline, distance_m=distance, samples=len(hits),
                minimum=min(hits), maximum=max(hits), median=float(np.median(hits)),
                no_hits=sum(x == 0 for x in hits), at_most_two=sum(x <= 2 for x in hits)))
    sources = [Path(__file__), ROOT/'scripts/validate_g1_traffic.py', source]
    result = dict(scope='stationary ideal ray geometry, not driving or hardware proof',
        source_hashes={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        lidar_height_m=.1346, phase_patterns=patterns, summary=summary, rows=rows)
    (A/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
