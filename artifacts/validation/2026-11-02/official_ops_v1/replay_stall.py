"""a5 정체 구간을 재획득 후보 ON으로 재생해 이유 변화를 본다."""
from collections import Counter
from dataclasses import replace
import json
import sys

from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig

path, start, end = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
header, calls = None, []
with open(path, encoding='utf-8') as stream:
    for line in stream:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event['kind'] == 'header':
            header = event['data']
        elif event['kind'] == 'call' and start <= (event['data']['input'].get('reference_s') or 0.) <= end:
            calls.append(event['data']['input'])
motion = MotionConfig(**header['motion_config'])
follow = lf.LocalFollow(replace(lf.FollowConfig(**header['follow_config']), standing_wall_reacquire=True))
reasons = Counter()
for index, call in enumerate(calls):
    speed, steering, detail = replay_call(follow, call, motion)
    reasons[detail['reason']] += 1
    if index < 8 or index % 10 == 0:
        keep = {k: detail.get(k) for k in ('reason', 'behavior', 'boundary_selection', 'bilateral_reference', 'boundary_candidate_reason',
                                           'boundary_candidate_ahead_m', 'fresh_boundary_accepted', 'path_wall', 'swept_clearance_m',
                                           'target_speed_before_hold_mps')}
        print(index, round(call['reference_s'], 2), round(speed, 3), round(steering, 3), follow.pending_count, follow.path is not None, keep)
print(reasons, len(calls))
