import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time
import types
import zipfile
import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
import candidate

A = Path(__file__).resolve().parent


def stats(values):
    return dict(count=len(values), p50_ms=statistics.median(values), p95_ms=sorted(values)[int(.95*len(values))])


def main():
    # 공통 코드에 채택된 뒤에도 비교 기준은 변경 전 보존 바이트여야 한다.
    before = A/'before_sources.zip'
    if before.exists():
        name = 'r172_preserved_pass_release'
        module = types.ModuleType(name); sys.modules[name] = module
        with zipfile.ZipFile(before) as archive:
            source = archive.read('src/arena_autonomy/arena_autonomy/pass_release_trial.py')
        exec(compile(source, str(before)+'!pass_release_trial.py', 'exec'), module.__dict__)
        original = module.threat_view
    else:
        original = ps.threat_view
    trace = A.parent/'released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as f:
        header = json.loads(next(f))['data']
        rows = [r for r in map(json.loads, f) if r['kind']=='call']
    motion = MotionConfig(**header['motion_config']); cfg = FollowConfig(**header['follow_config'])
    lat = ps.lateral_config(cfg); equal = 0; changed = []; old_cost = []; new_cost = []
    for row in rows:
        item = row['data']; inp = item['input']
        path = (item['output'][2].get('normal') or {}).get('path_points')
        _, scan, history = restore_call(inp, motion)
        scene = motion_scene(scan, history, inp['reference_s'], timing_verified=inp['timing_verified'],
                             epoch=inp['epoch'], rear_half_angle_deg=inp['rear_half_angle_deg'])
        answers = {}
        order = [('old', original, old_cost), ('new', candidate.threat_view, new_cost)]
        if row['seq'] % 2: order.reverse()
        for name, fn, costs in order:
            start = time.perf_counter(); value = fn(scene, path, lat)
            costs.append(1000*(time.perf_counter()-start)); answers[name] = value
        old, new = answers['old'], answers['new']
        same = old is None and new is None
        if old is not None and new is not None:
            same = old['state']==new['state'] and all(old[name]==new[name] for name,_ in ps.SIDES)
            probes = np.array([[0.,0.], [.4,.1],[-.2,-.1],[1.,.2]])
            same = same and all(np.array_equal(a,b) for a,b in zip(old['frame'].project(probes),new['frame'].project(probes)))
        equal += int(same)
        if not same: changed.append(row['seq'])
    report = dict(calls=len(rows), equal=equal, changed=changed, original=stats(old_cost), candidate=stats(new_cost),
                  scope='PC saved scenes, alternating order; no source changes, Jetson or drive execution')
    (A/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    return int(bool(changed))


if __name__=='__main__': raise SystemExit(main())
