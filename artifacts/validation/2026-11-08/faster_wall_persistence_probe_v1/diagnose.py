"""r32 실패의 현재 반환·구조 가림·실제 팽창을 다시 분리한다. 실행 소스 변경 없음."""
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
from arena_autonomy.known_structure import parse_sectors, classify
from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
targets = [11.2, 12.3, 13., 17.]
found = {}
header = None
for line in SOURCE.open():
    event = json.loads(line)
    if event['kind'] == 'header':
        header = event['data']
    if event['kind'] != 'call':
        continue
    data = event['data']; t = data['input']['reference_s']
    requested = next((v for v in targets if v not in found and t >= v-1e-6), None)
    entry = data['output'][2].get('moving_pass', {}).get('entry', {})
    if requested is None or not entry.get('candidates'):
        continue
    row = entry['candidates'][0]
    if row.get('reason') != 'known_occupied_command_hold':
        continue
    cfg = FollowConfig(**header['follow_config'])
    original, scan, history = restore_call(data['input'], MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, t, timing_verified=original['timing_verified'],
        epoch=original['epoch'], rear_half_angle_deg=original['rear_half_angle_deg'])
    side = row['current_outer_wall']; wall = entry['reference']['walls'][side]
    padding, evidence = current_outer_and_rear_padding(scene, cfg, side, wall, raw_scan=scan, sectors=parse_sectors(cfg.known_structure_sectors_json))
    points = np.asarray(scene.occupied_endpoints_xy_m)
    indices = [r.source_index for r in scene.rays if r.status == 'return']
    base = .025+cfg.measurement_padding_m
    sign = 1 if side == 'left' else -1
    slope = wall['slope']; norm = math.hypot(1., slope)
    actual_intercept = wall['intercept_m']+sign*cfg.wall_offset_m*norm
    aligned = np.flatnonzero((abs(points[:,1]-slope*points[:,0]-actual_intercept)/norm <= .015)
                             & (points[:,0]>=-2.) & (points[:,0]<=1.5))
    source_ids = np.asarray(indices)
    cuts = ((np.diff(source_ids[aligned]) != 1)
            | (np.linalg.norm(np.diff(points[aligned],axis=0),axis=1)>.10))
    fragments = []
    for piece in np.split(aligned,np.flatnonzero(cuts)+1):
        if len(piece)>=3:
            fragments.append(dict(count=len(piece),indices=[int(source_ids[piece[0]]),int(source_ids[piece[-1]])],
                x_span_m=float(np.ptp(points[piece,0])),x_bounds_m=[float(points[piece,0].min()),float(points[piece,0].max())]))
    _,_,structures = classify(scan,parse_sectors(cfg.known_structure_sectors_json))
    structural_gaps = [dict(indices=row['rays'].tolist(),explained=row['explained']) for row in structures]
    hits = []
    for angle in sorted({row['steering_rad'], original['previous_steering_rad']}):
        k = math.tan(angle)/cfg.wheelbase_m
        for distance in (0., .04):
            yaw = k*distance; c, s = math.cos(yaw), math.sin(yaw)
            rx, ry = ((distance-cfg.wheelbase_m/2, 0.) if abs(k)<1e-5 else
                      (s/k-cfg.wheelbase_m/2, (1-c)/k))
            dx, dy = points[:,0]-rx, points[:,1]-ry
            x, y = c*dx+s*dy-cfg.wheelbase_m/2, -s*dx+c*dy
            hit = (abs(x)<=cfg.length_m/2+padding)&(abs(y)<=cfg.width_m/2+padding)
            for i in np.flatnonzero(hit):
                hits.append(dict(point=points[i].tolist(), source_index=indices[i], padding_m=float(padding[i]),
                    fixed_wall=bool(padding[i] <= base+1e-10),
                    distance_from_fitted_wall_m=abs(float(points[i,1]-slope*points[i,0]-actual_intercept))/norm,
                    at_distance_m=distance, steering=angle))
    found[requested] = dict(t=t, speed=original['speed_mps'],command=data['output'][:2], selected=entry.get('selected'),target_steer=row['steering_rad'], previous_steer=original['previous_steering_rad'], hits=hits, evidence=evidence,
        wall=wall, fragments=fragments, structural_gaps=structural_gaps, reason=row['reason'], raw_current_body_hit_count=int(np.sum(
            (abs(points[:,0])<=cfg.length_m/2)&(abs(points[:,1])<=cfg.width_m/2))))
    if len(found) == len(targets):
        break
(HERE/'stop_fragments.json').write_text(json.dumps(found, ensure_ascii=False, indent=2)+'\n')
for requested, row in found.items():
    print(json.dumps(dict(requested=requested,t=row['t'],speed=row['speed'],steer=row['target_steer'],command=row['command'],hit_count=len(row['hits']),
        raw_hits=row['raw_current_body_hit_count'],fragments=row['fragments'],structural_gaps=row['structural_gaps']),ensure_ascii=False))
assert len(found) == len(targets)
