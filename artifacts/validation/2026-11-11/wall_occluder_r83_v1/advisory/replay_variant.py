"""후보 판정식(단절 기준 시선 깊이 .06)을 넣고 r81 저장 입력을 열린 고리로 재생한다. 저장소 쓰기 없음.

차량 응답이 기록과 달라지는 시점 이후의 입력은 실제와 다르므로, 결과는 '같은 입력에서 판단이
어떻게 달라지는가'만 뜻한다.
"""
import sys, json, pickle
from pathlib import Path
sys.dont_write_bytecode = True
ROOT = Path('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local')
SNAP = ROOT/'build/r81_frozen_diagnosis'
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(SNAP/'src/arena_autonomy'), str(SNAP/'src/arena_vehicle_interface'), str(SNAP/'scripts')]
import numpy as np
import arena_autonomy.local_path as lp
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call

def limit(a, b):
    return .10+.035*min(np.linalg.norm(a), np.linalg.norm(b))

def anchored(points, broken_unused, min_depth=.06, max_points=6, cross=.04, radial=.10):
    n = len(points)
    alive = np.ones(n, dtype=bool)
    joins = []
    for _ in range(12):
        idx = np.flatnonzero(alive)
        p = points[idx]
        if len(p) < 3:
            break
        gaps = np.linalg.norm(np.diff(p, axis=0), axis=1)
        lim = .10+.035*np.minimum(np.linalg.norm(p[:-1], axis=1), np.linalg.norm(p[1:], axis=1))
        broken = gaps > lim
        for b, _ in joins:
            pos = np.searchsorted(idx, b)
            if pos < len(broken) and idx[pos] == b:
                broken[pos] = False
        found = None
        for i in np.flatnonzero(broken):
            for k in range(1, max_points+1):
                for lo, hi in ((i+1, i+1+k), (i-k+1, i+1)):
                    if lo < 1 or hi >= len(p):
                        continue
                    cluster, before, after = p[lo:hi], p[lo-1], p[hi]
                    if np.linalg.norm(after-before) > 2*limit(before, after):
                        continue
                    if min(np.linalg.norm(before), np.linalg.norm(after))-np.max(np.linalg.norm(cluster, axis=1)) < min_depth:
                        continue
                    center = cluster.mean(axis=0)
                    los = center/max(1e-9, np.linalg.norm(center))
                    if np.ptp(cluster @ np.array([-los[1], los[0]])) > cross or np.ptp(cluster @ los) > radial:
                        continue
                    found = (lo, hi)
                    break
                if found:
                    break
            if found:
                break
        if not found:
            break
        lo, hi = found
        alive[idx[lo:hi]] = False
        joins.append((int(idx[lo-1]), int(idx[hi])))
    return ~alive, joins

lp.narrow_occluders = anchored
base = pickle.load(open(OUT/'clouds.pkl', 'rb'))['decisions']
trace = ROOT/'artifacts/validation/2026-11-11/native_nonreturn_r81_v1/two_actor_run1/run1/follow_decisions.jsonl'
rows = []
for line in trace.open(encoding='utf-8'):
    e = json.loads(line); d = e['data']
    if e['kind'] == 'header':
        motion = MotionConfig(**d['motion_config'])
        follow = BypassHandoffFollow(FollowConfig(**d['follow_config']),
                                     HandoffExecutionConfig.from_mapping(d['execution_config']), allow_simulation_trial=True)
    elif e['kind'] == 'reset':
        follow.reset()
    elif e['kind'] == 'call':
        t = d['input']['reference_s']
        out = replay_call(follow, d['input'], motion)
        n = out[2].get('normal', out[2]) if isinstance(out[2], dict) else {}
        rows.append(dict(t=t, speed_cmd=out[0], steer=out[1], baseline_reason=n.get('baseline_reason'),
                         reason=n.get('reason'), selection=n.get('boundary_selection'),
                         base_speed_cmd=base[t]['speed_cmd'], base_baseline_reason=base[t]['baseline_reason'],
                         base_selection=base[t]['selection'], measured_speed=d['input']['speed_mps']))
def summarize(lo, hi):
    sel = [r for r in rows if lo <= r['t'] < hi]
    def count(key, pred):
        return sum(1 for r in sel if pred(r[key]))
    def tally(key):
        out = {}
        for r in sel:
            out[str(r[key])] = out.get(str(r[key]), 0)+1
        return out
    return dict(decisions=len(sel),
                recorded_zero_cmd=count('base_speed_cmd', lambda v: abs(v) < 1e-9),
                candidate_zero_cmd=count('speed_cmd', lambda v: abs(v) < 1e-9),
                recorded_g1_reason=tally('base_baseline_reason'), candidate_g1_reason=tally('baseline_reason'),
                changed=sum(1 for r in sel if abs(r['speed_cmd']-r['base_speed_cmd']) > 1e-9 or r['baseline_reason'] != r['base_baseline_reason']))
first_change = next((r for r in rows if abs(r['speed_cmd']-r['base_speed_cmd']) > 1e-9 or r['baseline_reason'] != r['base_baseline_reason']), None)
report = dict(first_changed_decision=first_change, windows={f'{lo}-{hi}': summarize(lo, hi) for lo, hi in
              ((0, 20.0), (20.0, 43.6), (43.6, 48.3), (48.3, 65.0), (65.0, 1e9))})
(OUT/'replay_variant_result.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float)+'\n', encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, default=float))
