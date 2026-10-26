"""저장 입력의 최초 기록 차단을 분해한다. 제어 정책/입력은 변경하지 않는다."""
import argparse
import cProfile
import hashlib
import io
import json
from pathlib import Path
import pstats
import time

import numpy as np
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
import arena_autonomy.m3_execution_trial as mx


def inspect(path, end, destination):
    raw = path.read_bytes()
    events = [json.loads(x) for x in raw.decode('utf8').splitlines()]
    head = events[0]['data']
    controller = mx.M3ExecutionFollow(FollowConfig(**head['follow_config']),
        mx.ExecutionConfig(**head['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**head['motion_config'])
    profiler, rows, checks = cProfile.Profile(), [], []
    original = mx.release_gate

    def observe(record, scene, history, frame, state, grid, cfg, execution, stop):
        start = time.perf_counter()
        gate = original(record, scene, history, frame, state, grid, cfg, execution, stop)
        item = {k: v for k, v in gate.items() if k != 'record'}
        item.update(t=scene.t_ref_s, duration_ms=1000*(time.perf_counter()-start),
                    detected_at=record.detected_at_s, last_confirmed_at=record.last_confirmed_at_s)
        updated = gate['record']
        if updated is not None and updated.possible_xy_m is not None:
            rest = updated.possible_xy_m
            s, lateral = frame.project(rest)
            pad = mx.ray_padding(scene, cfg, execution.history_motion_bound_mps)
            free = mx.depth_support(scene, rest, pad)
            returned = mx._both_sides_returned(scene, rest)
            missing = rest[~returned & ~free]
            item.update(possible_bounds=[rest.min(axis=0).tolist(), rest.max(axis=0).tolist()],
                        not_bracketed_cells=int(np.sum(~returned & ~free)),
                        behind_cells=int(np.sum((s-state['s0']) < -cfg.length_m/2)),
                        laterally_near=int(np.sum(np.abs(lateral) < cfg.width_m/2)),
                        missing_bounds=([missing.min(axis=0).tolist(), missing.max(axis=0).tolist()]
                                        if len(missing) else None))
        checks.append(item)
        return gate

    mx.release_gate = observe
    try:
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                controller.reset()
                continue
            t = event['data']['input']['reference_s']
            if t > end+1e-8:
                break
            active = t >= end-1.3
            if active:
                profiler.enable()
            before = len(checks)
            start = time.perf_counter()
            result = replay_call(controller, event['data']['input'], motion)
            seconds = time.perf_counter()-start
            if active:
                profiler.disable()
            gate = (result[2].get('m3_lateral') or {}).get('release_gate')
            if gate:
                rows.append({'t':t, 'ms':seconds*1000, 'gate':gate, 'new_checks':len(checks)-before})
    finally:
        mx.release_gate = original
    destination.mkdir(exist_ok=False)
    profiler.dump_stats(str(destination/'profile.pstats'))
    stream = io.StringIO()
    pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumtime').print_stats(45)
    (destination/'profile.txt').write_text(stream.getvalue(), encoding='utf8')
    out = {'input':str(path), 'input_sha256':hashlib.sha256(raw).hexdigest(),
           'input_unchanged':hashlib.sha256(path.read_bytes()).hexdigest()==hashlib.sha256(raw).hexdigest(),
           'note':'cProfile and detailed query audit add cost; use run46 for uninstrumented timings',
           'rows':rows, 'checks':checks}
    (destination/'result.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n',encoding='utf8')
    print(json.dumps({'input':str(path),'calls':len(rows), 'checks':len(checks),
        'first_blocked':next((r for r in rows if r['gate']['memory_blocked']),None)},ensure_ascii=False))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(exist_ok=False)
    root=Path('artifacts/validation/2026-10-25/m3_execution_review_v1')
    for name,end in [('run17_observed_release_closed_loop',5.04),('run27_small_center_stop_restart',13.04)]:
        inspect(root/name/'run/follow_decisions.jsonl',end,args.output/name)
