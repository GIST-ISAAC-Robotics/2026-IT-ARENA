"""저장 실패의 점유 제한점을 원문 출력/입력 불변으로 계측. 주행 제어 변경 없음."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import freeze, equal_output, replay_call
from arena_autonomy.lidar_motion import MotionConfig

root = Path(__file__).resolve().parents[4]
source = root/'artifacts/validation/2026-10-13/g1_lidar_stress_v1/side_left_uniform_s0/follow_decisions.jsonl'
out = Path(__file__).with_name('occupancy_probe.json')
before = hashlib.sha256(source.read_bytes()).hexdigest()
events = [json.loads(line) for line in source.read_text().splitlines()]
cfg = lf.FollowConfig(**events[0]['data']['follow_config'])
motion = MotionConfig(**events[0]['data']['motion_config'])
follow = lf.LocalFollow(cfg)
original = lf.swept_limit
probes, rows, mismatches = [], [], []

def swept(points, angle, wheelbase, **kwargs):
    result = original(points, angle, wheelbase, **kwargs)
    if result <= cfg.resume_clearance_m:
        p = np.asarray(points)
        curvature = math.tan(angle)/wheelbase
        hits, distance = [], None
        for d in np.arange(0., kwargs['horizon']+.001, .04):
            yaw = curvature*d
            rx = d-wheelbase/2 if abs(curvature) < 1e-5 else math.sin(yaw)/curvature-wheelbase/2
            ry = 0. if abs(curvature) < 1e-5 else (1-math.cos(yaw))/curvature
            dx, dy = p[:, 0]-rx, p[:, 1]-ry
            x = math.cos(yaw)*dx+math.sin(yaw)*dy-wheelbase/2
            y = -math.sin(yaw)*dx+math.cos(yaw)*dy
            indices = np.flatnonzero((abs(x) <= cfg.length_m/2+kwargs['margin']) &
                                     (abs(y) <= cfg.width_m/2+kwargs['margin']))
            if len(indices):
                distance = float(d)
                hits = [dict(index=int(i), point=p[i].tolist()) for i in indices]
                break
        probes.append(dict(angle=angle, margin=kwargs['margin'], clearance=result,
                           first_collision_distance=distance, hits=hits,
                           static_padding_counterfactual=original(points, angle, wheelbase,
                               **{**kwargs, 'margin': .025+cfg.measurement_padding_m})))
    return result

with patch.object(lf, 'swept_limit', swept):
    for event in events[1:]:
        if event['kind'] == 'reset': follow.reset()
        if event['kind'] != 'call': continue
        probes = []
        data = event['data']
        result = freeze(replay_call(follow, data['input'], motion))
        if not equal_output(result, data['output']): mismatches.append(event['seq'])
        if probes:
            rows.append(dict(seq=event['seq'], input_time=data['input']['reference_s'],
                previous_angle=data['input']['previous_steering_rad'], measured_speed=data['input']['speed_mps'],
                source_first=result[2]['source_first_s'], reason=result[2]['reason'], probes=probes))
report = dict(scope='offline_diagnostic_only_static_padding_is_not_proposed_control',
    input_sha256=before, input_unchanged=before == hashlib.sha256(source.read_bytes()).hexdigest(),
    instrumented_replay_matched=not mismatches, mismatches=mismatches,
    reasons=dict(Counter(row['reason'] for row in rows)), rows=rows,
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
with out.open('x') as stream: json.dump(report, stream, indent=2, allow_nan=False)
print(json.dumps({k:v for k,v in report.items() if k != 'rows'}))
