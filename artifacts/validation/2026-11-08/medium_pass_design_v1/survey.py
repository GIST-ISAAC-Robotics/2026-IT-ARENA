"""추월 진입 저장 스캔의 긴 직선 관측 조사. 새 명령/물리 실행은 하지 않는다."""
import hashlib
import json
import math
from pathlib import Path
from closure import estimate_closure

import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.solo_straight_budget import wall_run
from arena_autonomy.bypass_handoff import front_patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1/follow_decisions.jsonl'
rows = []
with SOURCE.open(encoding='utf8') as f:
    header = json.loads(next(f))['data']
    cfg = FollowConfig(**header['follow_config'])
    motion = MotionConfig(**header['motion_config'])
    for line in f:
        event = json.loads(line)
        if event.get('kind') != 'call':
            continue
        q = event['data']
        out = q['output'][2]
        if out.get('reason') != 'moving_pass_entry':
            continue
        moving = out['moving_pass']
        proposal = moving['entry']
        if proposal.get('guide_mode') != 'new_observed_entry':
            continue
        data, scan, history = restore_call(q['input'], motion)
        scene = motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                             epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        patch, detail = front_patch(scene,'contiguous_three_v0')
        rays = [r for r in scene.rays if r.status == 'return']
        xy = np.array([r.hit_xy_m for r in rays])
        indices = np.array([r.source_index for r in rays])
        selected = proposal['selected']
        side = 'left' if selected['offset_m'] > 0 else 'right'
        wall = proposal['reference']['walls'][side]
        # Rotate only for the budget measurement; no point is removed from occupancy.
        theta = math.atan(wall['slope'])
        c, s = math.cos(theta),math.sin(theta)
        basis = np.array([[c,-s],[s,c]])
        rotated = (xy-[0,wall['intercept_m']]) @ basis
        extent = wall_run(rotated,indices,1 if side=='left' else -1,cfg)
        patch_end = float(np.max(patch[:,0]))
        # Fixed observed low-speed inputs are reused only to compare algebraic requirements.
        # The medium opponent speeds below are hypothetical, not measurements from this run.
        profiles = []
        for ego, lead in ((.65,moving['admission']['estimated_lead_upper_mps']), (1.1,.70), (1.1,.85), (1.1,1.05)):
            relative = ego-lead
            relative_gain = patch_end+math.hypot(.20,.15)+cfg.length_m/2+.06+cfg.measurement_padding_m
            t = relative_gain/relative if relative>0 else None
            distance = ego*t+selected['transition_m'] if t is not None else None
            profiles.append(dict(ego_cap_mps=ego,lead_upper_mps=lead,relative_gain_m=relative_gain,
                                 ideal_relative_time_s=t,ideal_forward_with_rejoin_m=distance,
                                 acceleration_plan=estimate_closure(min(data['speed_mps'],ego),ego,lead,relative_gain,selected['transition_m']),
                                 observed_outer_covers_ideal=bool(extent and distance is not None and extent[0]>=distance)))
        rows.append(dict(t=scene.t_ref_s,ego_measured_mps=data['speed_mps'],side=side,
                         chosen_offset_m=selected['offset_m'],wall_near_reference=wall['observed_x_m'],
                         extended_connected_outer_observation=extent,front_patch_end_m=patch_end,profiles=profiles))
        break
report = dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              current_runtime_horizon_unchanged=True,rows=rows,
              limitations=['medium speeds hypothetical, no medium driving',
                           'constant capped ego speed ignores acceleration and slowing',
                           'straight wall extent is geometry only, not obstacle-free route',
                           'future opponent acceleration and rejoin are not guaranteed'])
assert rows, 'no new-observed-entry found'
(HERE/'survey.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,ensure_ascii=False))
