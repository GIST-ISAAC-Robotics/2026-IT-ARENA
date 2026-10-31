"""완주 판정과 별개로 저장 영상 주행의 장시간 무진행을 검토한다."""
import collections
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.corridor_observer import current_proposal
from arena_autonomy.scene_motion import motion_scene


ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'run1'
OUT = ROOT / 'progress_review2'
OUT.mkdir(exist_ok=False)
files = [RUN/'trajectory.jsonl', RUN/'follow_decisions.jsonl']
hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
trajectory = [json.loads(s) for s in files[0].open(encoding='utf-8')]
active = [r for r in trajectory if 7.751 < r['t'] < 162.9]
windows = []
lo = 0
minimum, maximum = collections.deque(), collections.deque()
for hi, row in enumerate(active):
    progress = row['progress_m']
    while minimum and active[minimum[-1]]['progress_m'] >= progress:
        minimum.pop()
    while maximum and active[maximum[-1]]['progress_m'] <= progress:
        maximum.pop()
    minimum.append(hi); maximum.append(hi)
    while active[maximum[0]]['progress_m'] - active[minimum[0]]['progress_m'] > .05:
        lo += 1
        while minimum[0] < lo: minimum.popleft()
        while maximum[0] < lo: maximum.popleft()
    if row['t'] - active[lo]['t'] >= 2.5:
        windows.append((row['t']-active[lo]['t'], lo, hi))
episodes = []
for duration, first, last in sorted(windows, reverse=True):
    if any(first <= e['_last'] and last >= e['_first'] for e in episodes):
        continue
    part = active[first:last+1]
    episodes.append(dict(_first=first, _last=last, start_s=part[0]['t'], end_s=part[-1]['t'],
        duration_s=duration, progress_span_m=max(r['progress_m'] for r in part)-min(r['progress_m'] for r in part),
        start_progress_m=part[0]['progress_m'], end_progress_m=part[-1]['progress_m'],
        positive_safe_samples=sum(r['safe_speed_mps']>0 for r in part), samples=len(part),
        safety_reasons=dict(collections.Counter(r['safety_reason'] for r in part))))
episodes = [{k:v for k,v in e.items() if not k.startswith('_')} for e in episodes]
targets = iter([117.3,118.1,120.1,130.1,147.5,148.1])
target = next(targets)
snapshots = []
fig, axes = plt.subplots(2,3,figsize=(15,9))
for line in files[1].open(encoding='utf-8'):
    event = json.loads(line)
    if event['kind']=='header':
        cfg=FollowConfig(**event['data']['follow_config'])
        motion=MotionConfig(**event['data']['motion_config'])
    if event['kind']!='call':continue
    call=event['data']
    if call['input']['reference_s'] < target:continue
    data,scan,history=restore_call(call['input'],motion)
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    points=np.asarray(scene.occupied_endpoints_xy_m)
    proposal=current_proposal(scene,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],cfg)
    paths={}
    ax=axes.flat[len(snapshots)]
    ax.scatter(points[:,0],points[:,1],s=2,c='gray')
    for side,color in [('left','blue'),('right','orange')]:
        path,info=local_path(points,side,cfg.wall_offset_m)
        paths[side]={k:v for k,v in info.items() if k!='path_points'}
        if path is not None:ax.plot(path[:,0],path[:,1],c=color,label=side)
    ax.plot([-.1,.1,.1,-.1,-.1],[-.075,-.075,.075,.075,-.075],c='black')
    ax.set(xlim=(-.4,2),ylim=(-1,1),aspect='equal',title=f"t={data['reference_s']:.2f}s / {proposal['reason']}")
    ax.grid();ax.legend(loc='lower right')
    snapshots.append(dict(time_s=data['reference_s'],speed_mps=data['speed_mps'],
        output=call['output'][:2],normal_reason=(call['output'][2].get('normal') or {}).get('reason'),
        paths=paths,proposal=proposal))
    try:target=next(targets)
    except StopIteration:break
fig.tight_layout();fig.savefig(OUT/'late_boundary.png',dpi=150);plt.close(fig)
result=dict(scope='post-run progress review; not retroactive replacement of registered evaluation',
    progress_window_m=.05,minimum_duration_s=2.5,episodes=episodes,snapshots=snapshots,
    follow_config=asdict(cfg),raw_sha256=hashes,
    raw_unchanged=hashes=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
(OUT/'review.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(episodes=episodes,snapshots=[dict(time_s=s['time_s'],paths=s['paths'],reason=s['proposal']['reason']) for s in snapshots],raw_unchanged=result['raw_unchanged']),ensure_ascii=False))
