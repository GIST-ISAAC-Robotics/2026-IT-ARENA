"""완료된 오차 주행의 선택 시점에서 경계 기하만 다시 계산한다."""
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT / 'src/arena_autonomy'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corridor_observer import current_proposal
from arena_autonomy.corner_corridor import short_prefix_evidence
from arena_autonomy.local_path import local_path

trace = BASE.parent / 'rotation_stress_v1/run1_course/follow_decisions.jsonl'
output = BASE.parent / 'rotation_stress_v1/run1_geometry_v1.json'
if output.exists():
    raise FileExistsError(output)
with trace.open('rb') as stream:
    before = hashlib.file_digest(stream, 'sha256').hexdigest()
targets = [65.94, 66.32, 67.22, 80.02]
collected = []
with trace.open(encoding='utf-8') as stream:
    for line in stream:
        event = json.loads(line)
        if event['kind'] == 'header':
            config = FollowConfig(**event['data']['follow_config'])
            motion = MotionConfig(**event['data']['motion_config'])
        if event['kind'] != 'call' or event['data']['input']['reference_s'] < targets[len(collected)] - 1e-9:
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=True,
            epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        points = scene.occupied_endpoints_xy_m
        proposal = current_proposal(scene, data['speed_mps'], data['side'],
            data['previous_steering_rad'], data['dt_s'], config, partial_candidate=True)
        short = short_prefix_evidence(points, data['side'], config)
        paths = {}
        for side in ('left', 'right'):
            path, info = local_path(points, side, config.wall_offset_m)
            paths[side] = dict(path=path.tolist() if path is not None else None, info=info)
        row = dict(reference_s=data['reference_s'], recorded_command=event['data']['output'][:2],
            recorded_reason=event['data']['output'][2].get('reason'),
            proposed=proposal, short_prefix=short, paths=paths, current_points=[list(p) for p in points])
        collected.append(row)
        print(json.dumps(dict(reference_s=row['reference_s'], recorded_reason=row['recorded_reason'],
            proposal_reason=proposal['reason'], short_prefix_reason=short['reason'],
            left_info=paths['left']['info'], right_info=paths['right']['info']), ensure_ascii=False), flush=True)
        if len(collected) == len(targets):
            break
with trace.open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == before
with output.open('x', encoding='utf-8') as stream:
    json.dump(dict(scope='stateless current geometry only; temporal confirmation and commands not replayed',
        input_sha256=before, input_unchanged=True, rows=collected), stream, ensure_ascii=False, indent=2, allow_nan=False)
