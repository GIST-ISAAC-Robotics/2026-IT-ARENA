"""E 스트레스(평가 전용 저장 재생): 미확정 갈림길 ID20/ID30 갠트리에 신호와 같은 STL 단면·610 mm 간격 지주가 있다고 가정.

마커 자세(scene.json)를 갠트리 중심으로 보고 도로 진행 방향 직각으로 ±0.3725 m에 15×14.62 mm 지주를 둔다(가정·실물 미확정).
r25 저장 입력에 반환을 합성해 후보 OFF/ON 계획기 판단을 비교한다. 열린 고리이며 실제 주행이 아니다.
"""
import json, math, sys
import numpy as np
from replay_tools import HERE, R25, Poses, build_controller, digest, finite_ranges, ray_box, read_events, scan_geometry
FLAGS = dict(narrow_occluder_bridge=True, junction_continuity=True)
# (마커 x, y, 도로 방향 yaw): ID20은 -x 진행(가로대가 y축), ID30은 -y 진행(가로대가 x축)
GANTRIES = {20: (6.155, 6.0, 0.0), 30: (2.48, 3.505, math.pi/2)}
def boxes():
    out = []
    for marker, (x, y, road_yaw) in GANTRIES.items():
        nx, ny = -math.sin(road_yaw), math.cos(road_yaw)
        for sign in (1, -1):
            out.append((x+sign*.3725*nx, y+sign*.3725*ny, road_yaw, .01462, .015))
    return out
def main():
    from arena_autonomy.follow_trace import freeze, replay_call
    source = R25/'follow_decisions.jsonl'; before = digest(source); poses = Poses(); bx = boxes(); cache = {}
    rows = []; runs = None
    for e in read_events(source):
        d = e['data']
        if e['kind'] == 'header':
            runs = {'plain_off': build_controller(d), 'off': build_controller(d),
                    'on': build_controller(dict(d, follow_config={**d['follow_config'], **FLAGS}))}; continue
        if e['kind'] == 'reset':
            for c, _ in runs.values(): c.reset()
            continue
        if e['kind'] != 'call': continue
        scan = d['input']['scan']; key = (scan['sec'], scan['nanosec'])
        if key not in cache:
            ox, oy, dx, dy = scan_geometry(scan, poses)
            post = np.min([ray_box(ox, oy, dx, dy, *b) for b in bx], axis=0); r = finite_ranges(scan['ranges'])
            rep = np.isfinite(post) & np.isfinite(r) & (post < r) & (post <= scan['range_max'])
            cache[key] = ([float(post[i]) if rep[i] else v for i, v in enumerate(scan['ranges'])], int(rep.sum()))
        mod = json.loads(json.dumps(d['input'])); mod['scan']['ranges'] = cache[key][0]
        o = {k: freeze(replay_call(c, d['input'] if k == 'plain_off' else mod, mc)) for k, (c, mc) in runs.items()}
        rows.append(dict(t=d['input']['reference_s'], rays=cache[key][1], plain=float(o['plain_off'][0]), off=float(o['off'][0]), on=float(o['on'][0])))
    assert digest(source) == before
    win = [r for r in rows if r['t'] > 8.4]
    z = lambda k: sum(r[k] <= 1e-6 for r in win)
    s = dict(assumption='unconfirmed junction gantry posts at ID20/ID30 marker poses, STL sections, 610 mm spacing',
             posts=[[round(v, 4) for v in b[:3]] for b in bx], calls=len(rows), calls_with_post_rays=sum(r['rays'] > 0 for r in rows),
             zero_plain=z('plain'), zero_off=z('off'), zero_on=z('on'),
             mean_plain=float(np.mean([r['plain'] for r in win])), mean_off=float(np.mean([r['off'] for r in win])), mean_on=float(np.mean([r['on'] for r in win])),
             scope='open-loop r25 stored inputs; stress only; not a closed-loop run')
    (HERE/'e_junction_posts_summary.json').write_text(json.dumps(s, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(s, indent=1))
if __name__ == '__main__': sys.exit(main())
