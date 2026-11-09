"""지주 합성 재생에서 감속한 판단의 세부 차이(평가 전용)."""
import json, sys
import numpy as np
from replay_tools import R25, Poses, build_controller, finite_ranges, ray_box, read_events, scan_geometry
from posts_replay import post_boxes
from arena_autonomy.follow_trace import replay_call, freeze

WINDOWS = [(9.6, 10.35), (37.1, 37.65)]
poses = Poses(); boxes, _ = post_boxes()
base = inj = mc = None
def flat(d, prefix=''):
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(flat(v, prefix + k + '.'))
        elif not isinstance(v, list):
            out[prefix + k] = v
    return out
report = []
for event in read_events(R25 / 'follow_decisions.jsonl'):
    data = event['data']
    if event['kind'] == 'header':
        base, mc = build_controller(data); inj, _ = build_controller(data); continue
    if event['kind'] == 'reset':
        base.reset(); inj.reset(); continue
    if event['kind'] != 'call':
        continue
    scan = data['input']['scan']
    ox, oy, dx, dy = scan_geometry(scan, poses)
    post = np.min([ray_box(ox, oy, dx, dy, *b) for b in boxes], axis=0)
    ranges = finite_ranges(scan['ranges'])
    rep = np.isfinite(post) & np.isfinite(ranges) & (post < ranges) & (post <= scan['range_max'])
    modified = json.loads(json.dumps(data['input']))
    modified['scan']['ranges'] = [float(post[i]) if rep[i] else v for i, v in enumerate(scan['ranges'])]
    a = freeze(replay_call(base, data['input'], mc)); b = freeze(replay_call(inj, modified, mc))
    t = data['input']['reference_s']
    if any(lo <= t <= hi for lo, hi in WINDOWS) and abs(a[0] - b[0]) > .05:
        fa, fb = flat(a[2]), flat(b[2])
        keys = sorted(k for k in set(fa) | set(fb) if fa.get(k) != fb.get(k) and not k.endswith('_s') and 'cost' not in k and 'wall_time' not in k)
        angles = np.degrees(scan['angle_min'] + np.flatnonzero(rep) * scan['angle_increment'])
        report.append(dict(t=t, base=a[0], inj=b[0], post_rays=int(rep.sum()), post_angles_deg=[round(x, 1) for x in angles],
                           post_ranges=[round(float(post[i]), 3) for i in np.flatnonzero(rep)],
                           diff={k: [fa.get(k), fb.get(k)] for k in keys[:60]}))
json.dump(report, open('posts_detail.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1, default=str)
for r in report[:3] + report[-3:]:
    print(json.dumps(r, ensure_ascii=False, default=str)[:3000])
print(len(report))
