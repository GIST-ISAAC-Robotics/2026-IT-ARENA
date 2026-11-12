"""저장 GUARD 거절의 측방 점을 평가 정답과 사후 비교한다. 제어 변경/발행 없음."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import Point
from shapely.ops import unary_union

from arena_autonomy.current_path_recheck import _geometry
from arena_autonomy import pass_release_trial as selector
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from build_experimental_track import obstacle_polygons

A = Path(__file__).resolve().parent
RUN = A.parent/'adjacent_low_runtime_r180_v1/two_actor_run1/run1'


def main():
    trace = RUN/'follow_decisions.jsonl'
    trajectory = [json.loads(s) for s in (RUN/'trajectory.jsonl').read_text().splitlines()]
    actor = [json.loads(s) for s in (RUN/'actor_poses.jsonl').read_text().splitlines()]
    extra = json.loads((RUN/'additional_actors.json').read_bytes())['poses']
    obstacles = unary_union(obstacle_polygons(RUN/'runtime_world.sdf'))
    rows = []
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        cfg = FollowConfig(**header['follow_config'])
        motion = MotionConfig(**header['motion_config'])
        for event in map(json.loads, stream):
            if event['kind'] != 'call':
                continue
            d = event['data']; current = d.get('current_rebase')
            if not current or current['output']['reason'] != 'guard_current_approach_not_observed':
                continue
            now = current['input']; t = now['reference_s']
            _, scan, history = restore_call(now, motion)
            scene = motion_scene(scan, history, t, timing_verified=now['timing_verified'],
                                 epoch=now['epoch'], rear_half_angle_deg=now['rear_half_angle_deg'])
            # 원 재검사의 동일 경로 재배치를 사용한다. 평면 정답은 아래 평가에만 쓴다.
            result = _geometry.rebase(
                (d['output'][0], d['output'][1], {**d['output'][2], 'normal': {
                    **d['output'][2]['normal'], 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}}),
                d['input'], now, scene, history, cfg)
            if result['speed_mps'] <= 0:
                raise AssertionError((event['seq'], result['reason']))
            lat = selector.lateral_config(cfg)
            view = selector.threat_view(scene, result['path_points'], lat)
            grid = selector.mx.ReturnGrid(scene, lat)
            _, masks = selector.mx.side_return_masks(view['frame'], view['state'], grid, lat)
            ego = min(trajectory, key=lambda x: abs(x['t']-t))
            c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
            rotation = np.array([[c, -s], [s, c]])
            actor_rows = {'primary': min(actor, key=lambda x: abs(x[0]-t))}
            actor_rows.update({name: min(poses, key=lambda x: abs(x[0]-t))
                               for name, poses in extra.items()})
            sides = {}
            for (name, _), mask in zip(selector.SIDES, masks):
                points = grid.returns[mask]
                world = points@rotation.T + np.array([ego['x'], ego['y']])
                distances = [obstacles.distance(Point(p)) for p in world]
                sides[name] = dict(points=len(points), world_xy_m=world.tolist(),
                    within_30mm_of_static_collision=sum(v <= .03 for v in distances),
                    static_distance_max_m=max(distances, default=None),
                    minimum_actor_center_distance_m={actor_name: float(np.min(np.linalg.norm(
                        world-np.array(pose[1:3]), axis=1))) if len(world) else None
                        for actor_name, pose in actor_rows.items()})
            rows.append(dict(seq=event['seq'], reference_s=t, ego_truth_age_s=abs(t-ego['t']),
                ego_xy_yaw=[ego['x'],ego['y'],ego['yaw']], sides=sides,
                actor_truth_age_s={name:abs(pose[0]-t) for name,pose in actor_rows.items()}))
    result = dict(scope='saved-input evaluation only; nearest ego truth sample and static box collisions; no controller change',
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(), calls=len(rows), rows=rows)
    with (A/'result.json').open('x', encoding='utf8') as out:
        json.dump(result, out, indent=2);out.write('\n')
    print(json.dumps(dict(calls=len(rows), sides={name:dict(
        points=sum(r['sides'][name]['points'] for r in rows),
        near_static=sum(r['sides'][name]['within_30mm_of_static_collision'] for r in rows))
        for name,_ in selector.SIDES})))


if __name__ == '__main__':
    main()
