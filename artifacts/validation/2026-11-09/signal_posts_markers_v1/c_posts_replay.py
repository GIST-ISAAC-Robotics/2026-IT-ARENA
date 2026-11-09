"""C 사전 점검: r18 정지차 우회 저장 입력(신호 1 m 앞 정지차)에 지주 반환을 합성해 같은 계획기 판단을 비교(평가 전용, 열린 고리)."""
import json, sys
import numpy as np
from replay_tools import HERE, ROOT, Poses, build_controller, digest, finite_ranges, ray_box, read_events, scan_geometry
from posts_replay import post_boxes
RUN = ROOT / 'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1'
def main():
    from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
    source = RUN / 'follow_decisions.jsonl'; before = digest(source)
    poses = Poses(RUN / 'ego_poses.jsonl'); boxes, _ = post_boxes(); cache = {}; rows = []; base = inj = None
    for e in read_events(source):
        d = e['data']
        if e['kind'] == 'header':
            base = build_controller(d); inj = build_controller(d); continue
        if e['kind'] == 'reset':
            base[0].reset(); inj[0].reset(); continue
        if e['kind'] != 'call': continue
        scan = d['input']['scan']; key = (scan['sec'], scan['nanosec'])
        if key not in cache:
            ox, oy, dx, dy = scan_geometry(scan, poses)
            post = np.min([ray_box(ox, oy, dx, dy, *b) for b in boxes], axis=0)
            r = finite_ranges(scan['ranges'])
            rep = np.isfinite(post) & np.isfinite(r) & (post < r) & (post <= scan['range_max'])
            cache[key] = ([float(post[i]) if rep[i] else v for i, v in enumerate(scan['ranges'])], int(rep.sum()))
        mod = json.loads(json.dumps(d['input'])); mod['scan']['ranges'] = cache[key][0]
        a = freeze(replay_call(base[0], d['input'], base[1])); b = freeze(replay_call(inj[0], mod, inj[1]))
        da = a[2] if isinstance(a[2], dict) else {}; db = b[2] if isinstance(b[2], dict) else {}
        rows.append(dict(t=d['input']['reference_s'], post_rays=cache[key][1], base=float(a[0]), inj=float(b[0]),
                         base_steer=float(a[1]), inj_steer=float(b[1]), base_beh=da.get('behavior'), inj_beh=db.get('behavior'),
                         inj_reason=db.get('reason'), same=equal_output(comparable_decision(a), comparable_decision(b)),
                         base_rec=equal_output(comparable_decision(a), comparable_decision(freeze(d['output'])))))
    assert digest(source) == before
    beh = lambda k: sorted({r[k] for r in rows if r['post_rays']})
    s = dict(source=source.relative_to(ROOT).as_posix(), source_sha256=before, calls=len(rows),
             base_equals_recorded=sum(r['base_rec'] for r in rows), calls_with_post_rays=sum(r['post_rays'] > 0 for r in rows),
             changed=sum(not r['same'] for r in rows), zero_base=sum(r['base'] <= 1e-6 for r in rows), zero_inj=sum(r['inj'] <= 1e-6 for r in rows),
             behaviors_base_with_posts_visible=beh('base_beh'), behaviors_inj_with_posts_visible=beh('inj_beh'),
             bypass_calls_base=sum(r['base_beh'] == 'BYPASS' for r in rows), bypass_calls_inj=sum(r['inj_beh'] == 'BYPASS' for r in rows),
             first_post_ray_t=next((r['t'] for r in rows if r['post_rays']), None),
             max_abs_steer_delta=max(abs(r['inj_steer'] - r['base_steer']) for r in rows),
             scope='open-loop r18 stored inputs with synthetic STL-derived posts; recorded trajectory not re-simulated')
    (HERE / 'c_posts_replay_summary.json').write_text(json.dumps(s, indent=1) + '\n', encoding='utf-8', newline='\n')
    with open(HERE / 'c_posts_replay_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows: f.write(json.dumps(r) + '\n')
    print(json.dumps(s, indent=1))
if __name__ == '__main__': sys.exit(main())
