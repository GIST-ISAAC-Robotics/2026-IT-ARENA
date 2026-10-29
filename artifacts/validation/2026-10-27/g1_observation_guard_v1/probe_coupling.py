"""고정 자세 광선: 허용 결측과 실제 측방 위협의 결합을 현행 G1에서 비교. 명령대로 이동하지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts', 'tests'):
    sys.path.insert(0, str(ROOT/part))
from arena_autonomy.follow_trace import freeze
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history, scan
import numpy as np


def raw_scene(first, boxes, missing=()):
    raw = scan(first)
    result = []
    for a in raw.angle_min+np.arange(500)*raw.angle_increment:
        c, s = math.cos(a), math.sin(a)
        value = min(.425/max(abs(s), 1e-12), 8.)
        for x0, x1, y0, y1 in boxes:
            lo, hi = 0., value
            for origin, d, b0, b1 in ((.06, c, x0, x1), (0., s, y0, y1)):
                if abs(d) < 1e-12:
                    if not b0 <= origin <= b1:
                        lo, hi = 1., 0.
                    continue
                t0, t1 = sorted(((b0-origin)/d, (b1-origin)/d))
                lo, hi = max(lo, t0), min(hi, t1)
            if lo <= hi and lo > 0:
                value = min(value, lo)
        if abs(a) > math.radians(150)+1e-12:
            value = math.inf
        result.append(value)
    for i in missing:
        result[i] = math.inf
    raw.ranges = result
    return raw


def config(selector):
    return FollowConfig(partial_observation_policy='bounded_local_v4',
                        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
                        corridor_fallback_enabled=True, parallel_hold_trial=selector,
                        pass_release_trial=selector, guard_curve_trial=selector)


def probe():
    h = history(end=4.)
    cases = {'clear': ([], ()), 'side': ([(-.05, .15, .22, .37)], ()),
             'gap': ([], (125,)), 'side_gap': ([(-.05, .15, .22, .37)], (125,)),
             'side_wide': ([(-.05, .15, .22, .37)], tuple(range(118, 130))),
             'side_gap_front_blocker': ([(-.05, .15, .22, .37), (.25, .45, -.075, .075)], (125,))}
    rows = []
    for name, (boxes, missing) in cases.items():
        for enabled in (False, True):
            ctl = LocalFollow(config(enabled))
            results = []
            for i in range(25):
                first = .2+i*.1
                # 먼저 통로와 옆차를 관측. 결측은 1.0초부터.
                raw = raw_scene(first, boxes, missing if i >= 8 else ())
                scene = motion_scene(raw, h, first+.1, timing_verified=True)
                out = ctl.command(scene, h, 0., 'left', 0., .02, raw_scan=raw)
                d = out[2]
                results.append({'t': first+.1, 'speed': out[0], 'steering': out[1], 'reason': d.get('reason'),
                                'partial': d.get('partial_observation'), 'selector': d.get('selector_mode'),
                                'choice': d.get('selector_choice'), 'selector_reason': d.get('selector_reason'),
                                'parallel': (d.get('parallel_hold_trial') or {}).get('reason'),
                                'guard': d.get('selector_guard_candidate')})
            rows.append({'case': name, 'selector_enabled': enabled, 'calls': results,
                         'after_gap_positive': sum(r['speed'] > 0 for r in results[8:]), 'last': results[-1]})
    return rows


if __name__ == '__main__':
    out = Path(sys.argv[1])
    if out.exists():
        raise FileExistsError(out)
    paths = ['src/arena_autonomy/arena_autonomy/'+n+'.py' for n in ('local_follow', 'parallel_lane_trial', 'pass_release_trial', 'partial_observation')]
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
    rows = probe()
    out.write_text(json.dumps(freeze({'scope': 'fixed pose, real synthesized rays, no closed-loop movement or ROS protection',
                                      'source_sha256': hashes, 'rows': rows}), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps([{k:r[k] for k in ('case','selector_enabled','after_gap_positive','last')} for r in rows], ensure_ascii=False))
