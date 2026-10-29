"""고정40 s 입력의 예측 외곽/후반 분류 비용만 분리한다. 실시간 성능 판정 아님."""
import cProfile
import hashlib
import io
import json
from pathlib import Path
import pstats
import time
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import motion_region, lateral_config
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
OUT = BASE/'geometry_profile'
OUT.mkdir(exist_ok=False)
trace = BASE.parent/'traffic_recovery_connection_v1/run1_early_hold_prefix.jsonl'
for line in trace.open(encoding='utf-8'):
    row = json.loads(line)
    if row['kind'] == 'header':
        header = row['data']
    if row['kind'] == 'call' and abs(row['data']['input']['reference_s']-40.) < 1e-7:
        call = row['data']
        break
cfg = FollowConfig(**header['follow_config'])
data, raw, history = restore_call(call['input'], MotionConfig(**header['motion_config']))
scene = motion_scene(raw, history, data['reference_s'], timing_verified=data['timing_verified'],
                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
meta = call['output'][2]
_, steer, _ = pursue_path(np.asarray(meta['path_points']), data['speed_mps'], cfg, meta['boundary_age_s'])
steer = float(np.clip(steer, data['previous_steering_rad']-3*data['dt_s'], data['previous_steering_rad']+3*data['dt_s']))
if meta.get('corridor_selected'):
    steer = meta['corridor_candidate']['steering_rad']


def region():
    context = {}
    result = motion_region(scene, data['speed_mps'], .35, steer, data['previous_steering_rad'],
                           data['dt_s'], cfg, lateral_config(cfg), context=context)
    return result, context


result, context = region()
queries = context['queries'][~context['free'] & ~context['masked']]
def fringe():
    return bounded_rear_fringe(scene, raw, queries, context['padding'], cfg.length_m, cfg.width_m)

cost = {}
for name, callback in [('region', region), ('fringe', fringe)]:
    callback()
    cpu, wall = [], []
    for _ in range(30):
        t, c = time.perf_counter_ns(), time.thread_time_ns()
        callback()
        cpu.append((time.thread_time_ns()-c)/1e6)
        wall.append((time.perf_counter_ns()-t)/1e6)
    cost[name] = dict(cpu_median_ms=float(np.median(cpu)), wall_median_ms=float(np.median(wall)),
                      cpu_max_ms=max(cpu), wall_max_ms=max(wall))
profile = cProfile.Profile()
profile.runcall(fringe)
profile.dump_stats(OUT/'fringe.pstats')
buf = io.StringIO()
pstats.Stats(profile, stream=buf).sort_stats('cumulative').print_stats(25)
(OUT/'fringe.txt').write_text(buf.getvalue(), encoding='utf-8')
report = dict(scope='one fixed frame; 30 calls; concurrent Gazebo; not full pipeline or real-time proof',
              input_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(), reference_s=data['reference_s'],
              unknown_queries=len(queries), region=result, fringe_result=fringe(), cost=cost)
(OUT/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(unknown_queries=len(queries), cost=cost)))
