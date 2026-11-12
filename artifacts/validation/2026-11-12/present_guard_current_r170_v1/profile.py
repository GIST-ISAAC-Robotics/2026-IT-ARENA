"""저장 대표에서 재계산 단계 비용만 계측. 동작/판정과 실물 실행을 바꾸지 않는다."""
import collections
import json
from pathlib import Path
import statistics
import time
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
import current as candidate

A = Path(__file__).resolve().parent
measurements = collections.defaultdict(list)


def instrument(module, name):
    original = getattr(module, name)
    def measured(*args, **kwargs):
        start = time.perf_counter()
        try:
            return original(*args, **kwargs)
        finally:
            measurements[name].append(1000*(time.perf_counter()-start))
    setattr(module, name, measured)


def main():
    trace = A.parent/'released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as f:
        header = json.loads(next(f))['data']
        rows = [r for r in map(json.loads, f) if r['kind'] == 'call' and candidate.supported(r['data']['output'])]
    selected = rows[::7]
    motion = MotionConfig(**header['motion_config']); cfg = FollowConfig(**header['follow_config'])
    for module, name in ((candidate.geometry, 'rebase'), (candidate.previous, 'current_forward_coverage'),
                         (candidate.ps, 'threat_view'), (candidate.ps, 'motion_region'),
                         (candidate.ps, 'side_extent'), (candidate.ps, 'observed_guard_approach')):
        instrument(module, name)
    answers = []
    for row in selected:
        item = row['data']; inp = item['input']
        _, scan, history = restore_call(inp, motion)
        scene = motion_scene(scan, history, inp['reference_s'], timing_verified=inp['timing_verified'],
                             epoch=inp['epoch'], rear_half_angle_deg=inp['rear_half_angle_deg'])
        start = time.perf_counter()
        result = candidate.rebase(item['output'], inp, inp, scene, history, cfg)
        measurements['total_current_rebase'].append(1000*(time.perf_counter()-start))
        answers.append(dict(seq=row['seq'], speed_mps=result['speed_mps'], reason=result['reason']))
    stats = {name: dict(calls=len(v), p50_ms=statistics.median(v), p95_ms=sorted(v)[int(.95*len(v))], sum_ms=sum(v))
             for name, v in measurements.items()}
    report = dict(sample_count=len(selected), stride=7, steps=stats, outputs=answers,
                  scope='PC WSL saved snapshots; excludes ROS scheduling, serialization, source restoration and current scene build; not Jetson timing')
    (A/'profile_wsl.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='outputs'}))


if __name__ == '__main__': main()
