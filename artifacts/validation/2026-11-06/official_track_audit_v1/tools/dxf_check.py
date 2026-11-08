"""venue_layout.dxf 간이 대조 (python -I tools/dxf_check.py). 표준 라이브러리만 사용, 원본은 읽기만 한다.

레이어별 엔티티 수, 문자(마커·그리드·출발선 표기)와 위치, WALLS 선분과 PDF 벽표(전사본) 끝점 대조.
"""
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

ART = Path(__file__).resolve().parents[1]
DXF = ART / 'extracted/output_final/venue_layout.dxf'
PDF = json.loads((ART / 'tools/pdf_tables_transcribed.json').read_text(encoding='utf-8'))


def pairs(path):
    lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    for i in range(0, len(lines) - 1, 2):
        yield lines[i].strip(), lines[i + 1].strip()


def entities(path):
    in_entities = False
    cur = None
    for code, val in pairs(path):
        if code == '2' and val == 'ENTITIES':
            in_entities = True
            continue
        if not in_entities:
            continue
        if code == '0':
            if cur:
                yield cur
            if val == 'ENDSEC':
                return
            cur = {'type': val, 'codes': defaultdict(list)}
        elif cur is not None:
            cur['codes'][code].append(val)


def main():
    counts = Counter()
    texts = []
    wall_segments = []
    layer_bbox = {}
    for e in entities(DXF):
        c = e['codes']
        layer = c.get('8', ['?'])[0]
        counts[(layer, e['type'])] += 1
        xs = [float(v) for v in c.get('10', [])]
        ys = [float(v) for v in c.get('20', [])]
        if xs and ys:
            b = layer_bbox.setdefault(layer, [math.inf, math.inf, -math.inf, -math.inf])
            b[0], b[1] = min(b[0], *xs), min(b[1], *ys)
            b[2], b[3] = max(b[2], *xs), max(b[3], *ys)
        if e['type'] in ('TEXT', 'MTEXT'):
            texts.append({'layer': layer, 'text': (c.get('1') or [''])[0], 'x': round(xs[0], 4) if xs else None,
                          'y': round(ys[0], 4) if ys else None})
        if layer == 'WALLS' and e['type'] == 'LINE':
            wall_segments.append(((float(c['10'][0]), float(c['20'][0])), (float(c['11'][0]), float(c['21'][0]))))
        if layer == 'WALLS' and e['type'] == 'LWPOLYLINE':
            pts = list(zip(map(float, c['10']), map(float, c['20'])))
            closed = int((c.get('70') or ['0'])[0]) & 1
            seq = pts + ([pts[0]] if closed else [])
            wall_segments += list(zip(seq[:-1], seq[1:]))
    # PDF 벽 선분이 DXF WALLS 선분 위에 있는지(끝점 거리)
    def point_seg(p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        t = max(0, min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / max(1e-12, dx * dx + dy * dy)))
        return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)
    wall_match = []
    for wid, mat, mm, a, b in PDF['walls_road_face_lines']:
        da = min(point_seg(a, s, t) for s, t in wall_segments) if wall_segments else None
        db = min(point_seg(b, s, t) for s, t in wall_segments) if wall_segments else None
        wall_match.append({'id': wid, 'start_dist_m': None if da is None else round(da, 4),
                           'end_dist_m': None if db is None else round(db, 4)})
    res = {'dxf_bytes': DXF.stat().st_size,
           'entity_counts': {f'{k[0]}/{k[1]}': v for k, v in sorted(counts.items())},
           'layer_bbox': {k: [round(v, 4) for v in b] for k, b in layer_bbox.items()},
           'texts': texts, 'wall_segments': len(wall_segments),
           'pdf_wall_endpoints_off_dxf_walls_gt_2mm': [w for w in wall_match if w['start_dist_m'] is None or max(w['start_dist_m'], w['end_dist_m']) > 0.002],
           'pdf_wall_endpoint_max_dist_m': max((max(w['start_dist_m'], w['end_dist_m']) for w in wall_match), default=None)}
    (ART / 'checks/dxf_check.json').write_text(json.dumps(res, ensure_ascii=False, indent=1) + '\n', encoding='utf-8', newline='\n')
    out = dict(res)
    out['texts'] = texts[:60]
    import sys
    sys.stdout.reconfigure(encoding='utf-8')
    print(json.dumps(out, ensure_ascii=False, indent=1)[:7000])


if __name__ == '__main__':
    main()
